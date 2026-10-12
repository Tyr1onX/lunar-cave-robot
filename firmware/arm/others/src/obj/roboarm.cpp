#include "roboarm.hpp"
#include "mathetics.hpp"


/* ============ 反馈入口（CAN 回调）→ 控制出口（控制任务） ============
 * UpdateFeedback()：解出编码器连续角，只在第一帧做限位对齐
 * UpdateReference()：规划/积分；ControlStep()：PID → MIT 下发
 * ================================================================== */

/* 反馈入口：CAN 回调里调用。先解电机反馈，再展开连续角并更新 angle.theta。
   注意：**不要**在这里写规划器/PID 状态（本函数运行在中断上下文）。 */
void Joint::UpdateFeedback(const uint8_t* data_)
{
	Motor_DM::UpdateFeedback(data_);  // 显式限定：本函数隐藏了基类同名函数
	if (!inited)
	{
		pos_unwrapped = UnwrapRadNear(pos, offset);
		pos_wrapped_last = pos;
		const float center = UnwrapRadNear(0.5f * (angle_limit.pos_max + angle_limit.pos_min), pos_unwrapped);
		const float half = 0.5f * (angle_limit.pos_max - angle_limit.pos_min);
		limit_min = center - half;
		limit_max = center + half;
		// 规划初始化放到控制任务的 UpdateReference 首次进入时：
		// 本函数由 CAN 回调调用，不要在这里写规划器状态
		inited = true;
	}
	else
	{
		pos_unwrapped += RadDiff(pos, pos_wrapped_last);
		pos_wrapped_last = pos;
	}
	angle.theta = mdh.theta_offset + pos_unwrapped - offset;
}

/* 把 target_pos 钳到关节限位内。
   限位区间跨度可能接近 2π（如 j1 的 [-1.7, 3.48]），若把两端各自展开会落到不同圈次、
   导致区间倒置。第一帧反馈中先对齐区间中心再按半跨度重建，此后区间固定。 */
void Joint::ApplyLimit()
{
	LimitSelf(target_pos, limit_max - Joint_N::limit_margin, limit_min + Joint_N::limit_margin);
}

void Joint::UpdateLimitState(const float feedback_pos_)
{
	const int previous = limit_side;
	const float lower = limit_min + Joint_N::limit_margin;
	const float upper = limit_max - Joint_N::limit_margin;
	if (feedback_pos_ >= upper) limit_side = 1;
	else if (feedback_pos_ <= lower) limit_side = -1;
	else if ((limit_side > 0 && feedback_pos_ <= upper - Joint_N::limit_hysteresis) ||
		(limit_side < 0 && feedback_pos_ >= lower + Joint_N::limit_hysteresis)) limit_side = 0;
	if (limit_side != previous) pid.Clear();
}

void Joint::SyncReference()
{
	velocity_planner.Reset(pos_ref, vel_ref, acc_ref);
	if (velocity_mode) target_pos = pos_ref;
	else target_dirty = true;
	pid.Clear();
}

void Joint::RebaseReference(const float feedback_pos_)
{
	pos_ref = Limit(feedback_pos_, limit_max - Joint_N::limit_margin, limit_min + Joint_N::limit_margin);
	vel_ref = acc_ref = 0.f;
	velocity_planner.Reset(pos_ref);
	target_dirty = true;
	hold_rebase_armed = false;
	pid.Clear();
}

/* 两种模式都受硬边界约束；触限后不保留外向位置差、速度、加速度或旧规划。
   实际位置已触限时还将参考锚定到反馈以内，反拨不用先消化历史位置误差。 */
void Joint::GuardReference(const float feedback_pos_)
{
	const float lower = limit_min + Joint_N::limit_margin;
	const float upper = limit_max - Joint_N::limit_margin;
	Route_Plan_N::motion_ref_s ref{pos_ref, vel_ref, acc_ref};
	bool changed = Route_Plan_N::ClampMotionPosition(ref, lower, upper) != 0;
	if (limit_side != 0)
	{
		const float anchor = Limit(feedback_pos_, upper, lower);
		if (limit_side * (ref.pos - anchor) > 0.f)
		{
			ref = {anchor, 0.f, 0.f};
			changed = true;
		}
		if (limit_side * ref.vel > 0.f || limit_side * ref.acc > 0.f)
		{
			ref.vel = ref.acc = 0.f;
			changed = true;
		}
		if (!velocity_mode && limit_side * (target_pos - anchor) >= 0.f)
		{
			// 只裁剪本段可执行目标；hold_pos 保留，离开限位保护后仍能返回原保持点。
			changed = changed || target_pos != ref.pos || ref.vel != 0.f || ref.acc != 0.f;
			target_pos = ref.pos;
			ref.vel = ref.acc = 0.f;
		}
	}
	pos_ref = ref.pos;
	vel_ref = ref.vel;
	acc_ref = ref.acc;
	if (changed) SyncReference();
}

/* 挂起后、失能前保存一次：静止保持/自动复位保留原目标；
   用户主动运动途中急停则保存这一刻的实际位置，不继续原来的远端目标。 */
void Joint::CaptureHoldPosition()
{
	if (!inited) return;
	const bool preserve_hold = hold_valid && !velocity_mode &&
		(restoring_position || (!target_dirty && planner.IsArrived()));
	if (!preserve_hold)
	{
		hold_pos = Limit(pos_unwrapped, limit_max - Joint_N::limit_margin, limit_min + Joint_N::limit_margin);
		hold_valid = true;
	}
}

/* 仅重置运动历史，保留已保存的位置。下一周期从最新反馈限速规划回去。
   只能在控制任务挂起时调用；初次上电尚无保存位置时仍就地保持。 */
void Joint::SoftStart()
{
	resume_pos = hold_pos;
	resume_valid = hold_valid;
	++reset_generation;
	pid.Clear();
	plan_inited = false;
	velocity_mode = false;
	velocity_cmd = 0.f;
	remote_active = false;
	remote_wait_neutral = hold_valid;
	restoring_position = true;
	hold_rebase_armed = false;
	position_pending = false;
	stream_request = false;   // 流式请求一并作废，恢复后等新的下行目标重新入场
	stream_mode = false;
	stream_vel = 0.f;
	stream_vel_goal = 0.f;
	stream_idle_cycles = 0U;
	target_dirty = false;
	limit_side = 0;
	pos_ref = pos_unwrapped;
	vel_ref = acc_ref = 0.f;
}

/* 绝对位置入口：只发布请求，在控制周期中对齐圈次、限位并重规划。
   终点速度固定为 0（到点停车），适合点动、航点、夹爪这类一次性目标。 */
void Joint::SetTarget(const float pos_)
{
	if (!std::isfinite(pos_)) return;
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	position_request = pos_;
	stream_request = false;
	position_pending = true;
	__set_PRIMASK(primask);
}

/* 流式目标入口：目标由上位机按固定频率连续刷新，希望参考“跟着走”而不是
   “每帧走到新点再刹停”。与 SetTarget 共用同一请求通道（最后一次写入生效），
   区别只在控制周期里选的规划函数：这一条允许终点速度非 0（取目标自身速度）。 */
void Joint::SetStreamTarget(const float pos_)
{
	if (!std::isfinite(pos_)) return;
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	position_request = pos_;
	stream_request = true;
	position_pending = true;
	__set_PRIMASK(primask);
}

/* 只在控制任务中调用。模式切换沿用当前参考，不把每一拍重置成实测值。 */
void Joint::SetVelocity(const float vel_)
{
	stream_mode = false;   // 遥控/速度指令接管：退出流式跟踪
	if (!velocity_mode) velocity_planner.Reset(pos_ref, vel_ref, acc_ref);
	velocity_mode = true;
	restoring_position = false;
	hold_rebase_armed = false;
	velocity_cmd = std::isfinite(vel_) ? Limit(vel_, fminf(remote_vel_max, pid.cfg.vel_max)) : 0.f;
	target_dirty = false;
}

void Joint::SetRemoteVelocity(const float ratio_, const bool sticks_neutral_)
{
	const float ratio = std::isfinite(ratio_) ? Limit(ratio_, 1.f) : 0.f;
	if (remote_wait_neutral)
	{
		if (ratio == 0.f && sticks_neutral_) remote_wait_neutral = false;
		return;
	}
	if (ratio != 0.f) remote_active = true;
	if (remote_active) SetVelocity(ratio * remote_vel_max);
}

/* 绝对位置模式沿用原三次多项式，从当前 (q_ref,v_ref) 到固定目标，终点速度为零。
   规划速度预算为 remote_vel_max × plan_vel_scale，受 PID 总速度上限约束；
   总加速度预算为 PID accel_max × plan_accel_scale，一部分分给曲线，一部分分给时钟调速。
   该模式只保证 C1，
   速度模式的 S 曲线和制动包络由 velocity_planner 独立负责。 */
void Joint::PlanToTarget()
{
	const float vel_max = fminf(remote_vel_max * Joint_N::plan_vel_scale, pid.cfg.vel_max);
	const float acc_max = Joint_N::plan_accel_scale * pid.cfg.accel_max;
	const float T = Route_Plan_N::CubicTimeByLimits(target_pos - pos_ref, vel_ref, 0.f,
		vel_max, acc_max * Joint_N::position_accel_fraction);
	// 接口顺序：目标位置、完成时间、目标速度、起始位置、起始速度
	planner.Plan(target_pos, T, 0.f, pos_ref, vel_ref);
	position_time_scale = 1.f; // 新曲线起始速度已经是实际时间尺度的 vel_ref
	const float peak_vel = Route_Plan_N::CubicPeaks(planner.coeff).vel;
	position_scale_rate = fminf(1.f / Joint_N::jerk_ramp_time,
		acc_max * (1.f - Joint_N::position_accel_fraction) / fmaxf(peak_vel, 1.e-6f));
}

/* 目标速度估计（只在控制任务调用，每周期一次）。它回答“目标自己跑多快”，
   会作为流式规划段的**终点速度**：
     · 上游新目标到达 → 用「位移 ÷ 变化间隔」的差商更新收敛目标：对阶梯状的
       下行数据这是精确导数，与上位机帧率无关；
     · 目标连续不变超过 stream_hold_time → 收敛目标置 0：上游停住或链路断掉，
       参考就此平滑收住，不会带着旧速度一直跑下去。
   stream_vel 以加速度预算为上限向收敛目标逼近，所以上游阶跃不会让终点速度
   一步跳到速度上限，停流时的减速度也不会超过规划加速度预算。 */
void Joint::UpdateStreamVelocity(const bool new_target_)
{
	if (!stream_mode)
	{
		stream_vel = 0.f;
		stream_vel_goal = 0.f;
		return;
	}
	if (new_target_)
	{
		const float elapsed = static_cast<float>(stream_idle_cycles + 1U) * Joint_N::ctrl_dt;
		const float vel_max = fminf(remote_vel_max * Joint_N::plan_vel_scale, pid.cfg.vel_max);
		stream_vel_goal = Limit((target_pos - stream_target_prev) / elapsed, vel_max);
		stream_target_prev = target_pos;
		stream_idle_cycles = 0U;
	}
	else if (stream_idle_cycles < 0xFFFFU)
	{
		++stream_idle_cycles;
		if (static_cast<float>(stream_idle_cycles) * Joint_N::ctrl_dt > Joint_N::stream_hold_time)
			stream_vel_goal = 0.f;
	}
	const float step = Joint_N::stream_vel_slew_scale * Joint_N::plan_accel_scale
		* pid.cfg.accel_max * Joint_N::ctrl_dt;
	stream_vel += Limit(stream_vel_goal - stream_vel, step);
}

/* 流式目标规划：与 PlanToTarget 的唯一区别是**终点速度不为 0** —— 取目标自身的
   运动速度估计 stream_vel。目标是匀速时三次曲线退化成一条直线（b2 = b3 = 0），
   参考以同一速度跟住目标，既没有“每段都刹停”的加减速抖动，稳态也没有滞后；
   目标停住后 stream_vel 收敛到 0，段自动变回“到点停车”。 */
void Joint::PlanToStreamTarget(const float feedback_pos_)
{
	const float vel_max = fminf(remote_vel_max * Joint_N::plan_vel_scale, pid.cfg.vel_max);
	const float acc_max = Joint_N::plan_accel_scale * pid.cfg.accel_max;
	const float acc_budget = acc_max * Joint_N::position_accel_fraction;
	/* 防甩开窗口：参考不允许跑到实测位置前面超过跟踪窗口。位置模式原有的
	   “轨迹时钟调速”在流式下会被每帧重规划反复清掉，这里用一次直接的窗口裁剪
	   代替；窗口再与关节限位取交集，两道都不会被绕过。 */
	const float anchor = Limit(feedback_pos_, limit_max - Joint_N::limit_margin,
		limit_min + Joint_N::limit_margin);
	const float window_lower = fmaxf(limit_min + Joint_N::limit_margin, anchor - Joint_N::tracking_window);
	const float window_upper = fminf(limit_max - Joint_N::limit_margin, anchor + Joint_N::tracking_window);
	/* 终点速度取目标速度估计；但收敛目标为 0 说明目标已经不再移动（上游停住或链路断掉），
	   旧的目标速度会把参考带过最后一个目标 —— 此时必须能在剩余距离内刹住，
	   用梯形速度曲线的同一判据限制 |v| ≤ sqrt(2·a·剩余距离)。 */
	float terminal_vel = stream_vel;
	if (stream_vel_goal == 0.f)
	{
		terminal_vel = Limit(terminal_vel, Sqrt(2.f * acc_max * Abs(target_pos - pos_ref)));
	}
	/* 会合点与段时长必须自洽：位移取「目标 + 目标速度 × T」，而 T 又由这个位移解出。
	   目标是匀速时这组方程的解正好是一条直线（b2 = b3 = 0）—— 加速度为零、稳态无滞后。
	   迭代两三次即收敛，每步通常只做一次可行性检查。
	   ⚠️ 不要把外推时长写死成 plan_time：那样 T 与实际位移不自洽，每拍会在“加速度
	   上限”那一支里来回跳（实测 T 在 0.06 s 与 0.44 s 之间摆动），稳态滞后反而被
	   放大到约 5 倍，而且一直顶在加速度上限上。 */
	float T = Route_Plan_N::plan_time;
	for (int i = 0; i < 3; ++i)
	{
		const float dq = Limit(target_pos + terminal_vel * T, window_upper, window_lower) - pos_ref;
		T = Route_Plan_N::CubicTimeByLimits(dq, vel_ref, terminal_vel, vel_max, acc_budget);
	}
	const float aim = Limit(target_pos + terminal_vel * T, window_upper, window_lower);
	planner.Plan(aim, T, terminal_vel, pos_ref, vel_ref);
	position_time_scale = 1.f; // 新曲线起始速度已经是实际时间尺度的 vel_ref（同 PlanToTarget）
	const float peak_vel = Route_Plan_N::CubicPeaks(planner.coeff).vel;
	position_scale_rate = fminf(1.f / Joint_N::jerk_ramp_time,
		acc_max * (1.f - Joint_N::position_accel_fraction) / fmaxf(peak_vel, 1.e-6f));
}

/* 跟踪慢时只减慢规划时钟，不回拉参考、不清速度/PID，也不丢失保存目标。
   半窗口以内正常推进，接近完整窗口时平滑减速等待反馈；调速仍有制动距离，
   此窗口是软调速阈值，固定关节限位继续由 GuardReference 硬保护。
   q=q(tau), s=d(tau)/dt：v=q'·s，a=q''·s²+q'·ds/dt，前馈必须一并换算。 */
void Joint::UpdatePositionReference(const float feedback_pos_)
{
	const float error = Abs(pos_ref - feedback_pos_);
	const float soft_window = 0.5f * Joint_N::tracking_window;
	const float requested_scale = Limit((Joint_N::tracking_window - error) / soft_window, 1.f, 0.f);
	const float previous_scale = position_time_scale;
	position_time_scale += Limit(requested_scale - previous_scale, position_scale_rate * Joint_N::ctrl_dt);
	const float scale_rate = (position_time_scale - previous_scale) / Joint_N::ctrl_dt;
	const auto ref = planner.Update(0.5f * (previous_scale + position_time_scale) * Joint_N::ctrl_dt);
	pos_ref = ref.pos;
	vel_ref = ref.vel * position_time_scale;
	acc_ref = ref.acc * position_time_scale * position_time_scale + ref.vel * scale_rate;
}

/* 所有关节在同一逻辑时刻生成参考；动力学解算随后读取本拍 q/v/a。 */
void Joint::UpdateReference()
{
	if (!inited) return;
	const uint32_t generation = reset_generation;
	// 初始化只改变规划起点；保存目标独立于本拍参考与反馈。
	if (!plan_inited)
	{
		plan_inited = true;
		velocity_mode = false;
		velocity_cmd = 0.f;
		remote_active = false;
		restoring_position = true;
		RebaseReference(pos_unwrapped);
		if (!hold_valid)
		{
			hold_pos = pos_ref; // 仅首次上电：尚未保存过目标，就地保持
			hold_valid = true;
		}
		target_pos = hold_pos;
	}
	// 原子取走目标请求（绝对位置与流式共用一条通道），避免跨任务把一次请求丢在清标志期间。
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const bool pending = position_pending;
	const float requested = position_request;
	const bool stream = stream_request;
	position_pending = false;
	__set_PRIMASK(primask);
	if (pending)
	{
		/* 圈次对齐：把请求折进【限位区间所在的那一圈】，而不是「离当前位置最近的那一圈」。
		   限位区间跨度恒 < 2π，所以至多有一个圈次落在区间内；「区间中心 + WrapRad」
		   正好找到它（区间内任一点离中心不到 π，必是 WrapRad 选中的那个）。
		   只按“离当前位置最近”折叠，在跨度 > π 的关节上会选错圈 —— j6 的区间是
		   [-3.433, 1.800]（跨度 5.233），站在 -3.0 请求 +1.5 时会被折成 -4.783（区间外），
		   再被 ApplyLimit 钳回 -3.431，表现成“往那一头怎么都不走 / 跑到反方向”；
		   流式模式下还会让目标差商凭空跳出 ±2π，把 stream_vel 估成一记大速度。
		   折完再 ApplyLimit：只有当请求根本不在限位区间（按 mod 2π 算）里时才钳到最近端点。 */
		const float limit_center = 0.5f * (limit_max + limit_min);
		target_pos = limit_center + WrapRad(requested - limit_center);
		ApplyLimit();
		hold_pos = target_pos;
		hold_valid = true;
		velocity_mode = false;
		remote_active = false;
		restoring_position = false;
		hold_rebase_armed = false;
		target_dirty = true;
		if (stream)
		{
			// 首次进入流式模式：以本帧目标为基准清零速度估计，
			// 不能把入场之前的目标差当成目标速度。
			if (!stream_mode)
			{
				stream_target_prev = target_pos;
				stream_idle_cycles = 0U;
				stream_vel = 0.f;
				stream_vel_goal = 0.f;
			}
			stream_mode = true;
		}
		else
		{
			stream_mode = false;
		}
	}
	UpdateStreamVelocity(pending && stream);
	const float feedback_pos = pos_unwrapped;
	UpdateLimitState(feedback_pos);
	if (!velocity_mode)
	{
		// 已到位后的外力推移才重建起点；回位途中跟踪滞后由时钟调速处理，不能反复清零。
		if (hold_rebase_armed && Abs(pos_ref - feedback_pos) > Joint_N::tracking_window)
			RebaseReference(feedback_pos);
		float execution_target = hold_pos;
		const float anchor = Limit(feedback_pos, limit_max - Joint_N::limit_margin, limit_min + Joint_N::limit_margin);
		if (limit_side != 0 && limit_side * (execution_target - anchor) >= 0.f) execution_target = pos_ref;
		if (execution_target != target_pos)
		{
			target_pos = execution_target;
			target_dirty = true;
		}
	}
	GuardReference(feedback_pos);
	if (velocity_mode)
	{
		const float a_max = Joint_N::plan_accel_scale * pid.cfg.accel_max;
		// 实际位置越界时，窗口仍锚定在合法区间，不能形成 lower > upper。
		const float anchor = Limit(feedback_pos, limit_max - Joint_N::limit_margin, limit_min + Joint_N::limit_margin);
		const float lower = fmaxf(limit_min + Joint_N::limit_margin, anchor - Joint_N::tracking_window);
		const float upper = fminf(limit_max - Joint_N::limit_margin, anchor + Joint_N::tracking_window);
		const float command = (limit_side * velocity_cmd > 0.f) ? 0.f : velocity_cmd;
		const auto ref = velocity_planner.Update(command, Joint_N::ctrl_dt,
			fminf(remote_vel_max, pid.cfg.vel_max), a_max, a_max / Joint_N::jerk_ramp_time, lower, upper);
		pos_ref = ref.pos;
		vel_ref = ref.vel;
		acc_ref = ref.acc;
		target_pos = pos_ref;  // 不另存一份原始摇杆速度的累计目标
		if (velocity_planner.saturated) pid.Clear();
	}
	else
	{
		/* 流式模式：只要目标速度估计还没收敛到 0（上游在动，或刚停住正在收敛），
		   就每周期重规划。不重规划的话段结束后 planner 会一直输出段终点速度
		   （vel_target = stream_vel ≠ 0），参考位置停住却持续给速度指令。
		   条件里另外两条同样不能省：
		     · 上一段还挂着非 0 终点速度 —— 估计收敛到 0 的那一拍可能刚好落在两个
		       条件之间，必须再规划一次把终点速度真正写成 0；
		     · 本段终点被跟踪窗口裁过（不等于上游目标）—— 不重规划的话参考会永远
		       停在窗口边缘（实测上游做一个 0.6 rad 阶跃，参考卡死在 0.35 = 窗口宽度）。 */
		const bool stream_following = stream_mode &&
			((stream_vel != 0.f) || (stream_vel_goal != 0.f) ||
			 (planner.cfg.vel_target != 0.f) ||
			 (Abs(planner.cfg.pos_target - target_pos) > 1.e-6f));
		if (target_dirty || stream_following)
		{
			target_dirty = false;
			if (stream_mode) PlanToStreamTarget(feedback_pos);
			else PlanToTarget();
		}
		UpdatePositionReference(feedback_pos);
	}
	GuardReference(feedback_pos);
	// 一个任务可能在函数中途挂起。恢复快照比这个旧周期的局部变量优先。
	const uint32_t commit_primask = __get_PRIMASK();
	__disable_irq();
	if (generation != reset_generation)
	{
		hold_pos = resume_pos;
		hold_valid = resume_valid;
		plan_inited = false;
		velocity_cmd = 0.f;
		remote_active = false;
		remote_wait_neutral = hold_valid;
		hold_rebase_armed = false;
	}
	else if (velocity_mode)
	{
		// 用户主动操作或其减速尾段才更新保持点；静止保持不再进入速度积分器。
		hold_pos = pos_ref;
		hold_valid = true;
		if (velocity_cmd == 0.f && vel_ref == 0.f && acc_ref == 0.f)
		{
			velocity_mode = false;
			remote_active = false;
			restoring_position = true;
			target_pos = hold_pos;
			target_dirty = true;
		}
	}
	else if (vel_ref == 0.f && acc_ref == 0.f && Abs(pos_ref - hold_pos) < 1.e-5f &&
		Abs(feedback_pos - hold_pos) < 0.5f * Joint_N::tracking_window)
	{
		hold_rebase_armed = true;
	}
	__set_PRIMASK(commit_primask);
}

/* 控制出口：UpdateReference → 全轴 Solve → 分组调用本函数。 */
void Joint::ControlStep()
{
	if (!inited)
	{
		Ctrl(Motor_DM_N::order_byte_e::enable);
		return;
	}
	if (!plan_inited) return;  // 挂起可能发生在两组发送之间；恢复先等下一拍重建参考
	// 发送前再取一致的反馈快照，覆盖规划后、CAN 分组发送前才触限的情况。
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const float feedback_pos = pos_unwrapped;
	const float feedback_vel = vel;
	const float motor_pos = pos;
	__set_PRIMASK(primask);
	UpdateLimitState(feedback_pos);
	GuardReference(feedback_pos);
	const auto inward = [this](const float value)
	{
		return limit_side > 0 ? fminf(value, 0.f) : fmaxf(value, 0.f);
	};
	// 位置环只负责「规划位置 - 实际位置」的偏差 → 纠偏速度
	float v_corr = pid.Update(pos_ref, feedback_pos, feedback_vel);
	if (limit_side != 0)
	{
		// 同步限幅历史，反向输入不能被旧的正向纠偏输出/积分拖住。
		pid.output = inward(pid.output);
		pid.output_prev = inward(pid.output_prev);
		pid.integral = inward(pid.integral);
		v_corr = pid.output;
	}
	// 速度指令 = 规划前馈速度 + 纠偏速度；前馈不受 PID 内部限幅约束，这里对总指令再收一次
	float v_cmd = Limit(vel_ref + v_corr, pid.cfg.vel_max);
	if (limit_side != 0) v_cmd = inward(v_cmd);
	// 前馈力矩 = 重力前馈(tg) + 惯性/科氏前馈(td，由 RoboArmKine 按参考加速度算出)
	//            + 速度误差补偿(tq_v = kq_vel × (v_cmd - vel))
	//  速度补偿为什么走 torque 字段：MIT 的 d 项权限是 kd_m × (v_cmd - v)，本质是速度环的有限增益，
	//  大惯量/大负载轴（j2）即使把速度误差顶到 vel_max 也换不到足够力矩、跟不上 v_cmd；
	//  在力矩字段按同一个速度误差再补一份比例力矩，等效提高速度环增益。
	//  限制：① 它比电机内部 kd 多了一拍 CAN + 一个任务周期的延迟，增益只能取 MIT kd 的一部分
	//  （起点 ~40%），取大同样会振（A3-2 把阻尼整体搬进力矩通道就是那个症状）；
	//  ② 与 tg/td 共用 tq_lim 限幅，速度误差大时补偿会挤占重力前馈份额，因此不能让正常工况就顶满量程。
	const float tq_lim = 2047.5f / facs.torque_to_bit;  // 力矩量程，避免 12-bit 字段绕回
	const float tq_v = kq_vel * (v_cmd - feedback_vel);
	float tq_ff = Limit(tg + td + tq_v, tq_lim);
	uint16_t kp_cmd = kp_pos;
	uint16_t kd_cmd = kp_vel;
	if (limit_side != 0)
	{
		/* 仅限速度指令符号不足以约束 MIT 内部 kd*(v_des-v_actual)：
		   实际正向内退回时，零速度指令的阻尼也会产生外向力矩。
		   触限期间将速度环作用折算到力矩字段，再单侧投影，关闭内部 kp/kd。
		   MIT kd 的 12bit 物理量程为 0..5 N*m*s/rad，与现有驱动协议一致。 */
		const float kd_gain = static_cast<float>(kp_vel) * (5.f / 4095.f);
		tq_ff = Limit(inward(inward(tq_ff) + kd_gain * (v_cmd - feedback_vel)), tq_lim);
		kp_cmd = kd_cmd = 0U;
		// 12bit 力矩没有精确零码：下限选择向内的正半 LSB，避免截断编码成外向负值。
		if (limit_side < 0) tq_ff = fmaxf(tq_ff, 1.f / facs.torque_to_bit);
	}
	// pos_des 发当前编码值：不把位置误差交给电机的位置环（kp_pos = 0 时该字段被电机忽略）
	// vel_des = 规划前馈 + 纠偏速度, kd = kp_vel（速度阻尼）, torque_ff = 重力前馈 + 速度补偿
	if (plan_inited) Ctrl(motor_pos, v_cmd, kp_cmd, kd_cmd, tq_ff);
}
