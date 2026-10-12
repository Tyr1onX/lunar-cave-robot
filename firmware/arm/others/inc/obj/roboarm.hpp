#pragma once

#include <array>
#include <functional>
#include "motor_dm.hpp"
#include "pid.hpp"
#include "route_planning.hpp"
#include "velocity_planning.hpp"

/* ============================ 机械臂控制框架 ============================
 * 三层，单向数据流（详见 docs/调用框架与命名规范.md）：
 *
 *   1) 输入层   UART 中断解析，控制周期读取整帧快照
 *                 └─ Remote_UpdateJointVelocity() → Joint_SetRemoteVelocity(ratio)
 *                     └─ Joint::SetRemoteVelocity()：摇杆映射 rad/s，零输入主动撤销旧速度
 *
 *   2) 控制层   roboARM_task（2ms 一轮）
 *                 ├─ 全轴 UpdateReference()   位置规划 / 速度 S 曲线及解析积分
 *                 ├─ RoboArmKine::Solve()       本周期参考 → 重力/惯性前馈
 *                 └─ Joint::ControlStep()
 *                      ├─ pid.Update()      → v_corr
 *                      └─ Ctrl(pos, vel_ref+v_corr, ...)   MIT 下发
 *
 *   3) 生命周期 Error_tasks（10ms 轮询 Remote_IsHoldRequested()）
 *                 ├─ 挂起：suspend(roboARM) + 失能
 *                 └─ 解挂起：清错 → Joint_SoftStartAll() → 使能 → resume
 *
 * 命名规范（全工程统一）：
 *   POD 结构体 xxx_s ｜ 类 XxxYyy ｜ 命名空间 Xxx_N ｜ 常量 lower_snake
 *   成员变量 lower_snake（不加 m_ 前缀）｜ 成员函数 PascalCase() ｜ 参数名尾部加 _
 *   例外：任务入口函数名必须与 CubeMX 生成的 Core/Src/freertos.c 一致，不能改
 * ====================================================================== */

namespace Joint_N
{
	inline constexpr uint8_t dof = 7;  // 关节数（全工程唯一定义处）

	struct angle_limit_s
	{
		float pos_max;
		float pos_min;
	};

	inline constexpr float ctrl_dt = 0.002f;         // 控制周期 500 Hz
	inline constexpr float plan_vel_scale = 1.8f;    // 规划速度上限相对遥控等效速度的倍率
	inline constexpr float plan_accel_scale = 0.75f; // 规划加速度上限 = 该轴 PID accel_max × 本系数
	inline constexpr float jerk_ramp_time = 0.05f;  // jerk_max = acc_max / 50ms
	inline constexpr float tracking_window = 0.35f; // 遥控参考的制动包络相对反馈允许超前 (rad)
	inline constexpr float position_accel_fraction = 0.5f; // 位置曲线用一半加速度预算，另一半留给进度调速
	inline constexpr float limit_margin = 0.002f;   // 两种模式共同在关节角度限位内预留的参考裕量 (rad)
	inline constexpr float limit_hysteresis = 0.001f; // 实测离开边界后再释放输出方向保护，向内指令不等待解锁

	/* ---- 流式目标（上位机连续下发、目标持续变化）-----
	 * 上位机按固定频率刷新目标时，不能再用“到点停车”的三次曲线（终点速度恒 0）：
	 * 每段都要求刹停，参考速度会反复加减速，表现为跟随抖动和大滞后。
	 * 流式模式的终点速度取目标自身的运动速度估计，匀速时曲线退化成直线。
	 *   stream_hold_time     目标连续不变的时长上限 (s)：超过它就认为上游已经停住，
	 *                        把目标速度估计收敛到 0（流断开时的安全网）。
	 *   stream_vel_slew_scale 目标速度估计的变化率上限 = 该系数 × 加速度预算，
	 *                        防止上游阶跃让终点速度一步跳到速度上限。 */
	inline constexpr float stream_hold_time = 0.10f;
	inline constexpr float stream_vel_slew_scale = 1.0f;

	/* 统一存放关节的角度信息 */
	struct angle_s
	{
		float theta;    // 关节角 θ = mdh.theta_offset + (编码器连续角 - offset)
		float base[3];  // 连杆单位向量在基座坐标系下的表示（由运动学解算写入）
	};
}

namespace RoboArm_N
{
	struct mdh_param_s
	{
		float alpha;
		float a;  // i - 1
		float theta_offset;
		float d;  // i
		float axis[3];  // 该关节对应连杆在自身局部坐标系中的方向向量（单位向量：x/y/z）
	};

	inline constexpr std::array<mdh_param_s, Joint_N::dof> mdh_left
	{{
		{0.f, 0.f, 0.f, 0.f, 
			{0.f, 0.f, 1.f}},             
		{1.57080f, 0.f, 2.65289f, 0.f, 
			{1.f, 0.f, 0.f}},    
		{3.14159f, 0.f, 4.22348f, 0.f, 
			{0.f, -1.f, 0.f}},    
		{1.57080f, 0.f, 3.14159f, 0.f, 
			{0.f, 0.f, 1.f}},        
		{1.57080f, 0.f, 4.57276f, 0.f, 
			{0.f, -1.f, 0.f}},   
		{1.57080f, 0.f, 0.f, 0.f, 
			{0.f, 0.f, 1.f}},         
		{0.f, 0.f, 0.f, 0.f, 
			{0.f, 0.f, 1.f}}               
	 }};

	/* 实测等效转动惯量（惯性前馈的对角项）—— 来自转动惯量测量/identify_inertia.cpp 的辨识。
	   模型只有“质心在杆中心的点质量”，实测值与模型差一个仿射关系，所以对角项直接用：
	     J_i(姿态) = kJ_scale[i] * M_ii_模型(姿态) + kJ_offset[i]
	   kJ_scale 无量纲；kJ_offset 单位 kg·m²，装模型里没有的常量惯量（底座回转体、电机转子、
	   线束、以及连杆偏置带来的交叉项）。修正位置见 roboarm_kinematics.cpp 的
	   AccumulateInertia()：只改对角项，非对角/科氏项仍由模型给出。

	   辨识方法：导出 CAN 回传的 pos/vel/torque 为 CSV，逐运动段最小二乘
	     tau = J*alpha + B*omega + Fc*sgn(omega) + b
	   再按质量筛选跨段回归（惯性项占扭矩 >25%、摩擦项 <80%）。两个关键点：
	     (1) 必须先用 -g 扣掉固件下发的重力前馈 tg，否则重力梯度会按 1/omega² 串进 J；
	     (2) alpha 必须由单圈角度 unwrap 后再拟合，不能用 vel 列直接微分（电机上报速度已被滤波）。

	   实测汇总（括号内为该姿态下的模型 M_ii）：
	     j1 | 3.30*M11 + 0.109 | 12 点、两次独立实验 R²=0.979 | 置信度高
	        折叠臂形实测 0.245~0.328（模型 0.062~0.067）；伸展时 0.435~1.287（模型 0.106~0.352）
	     j2 | 实测 0.22~0.29（模型 0.24~0.36）                    | 置信度中
	     j3 | 实测 0.218（模型 0.1881）→ 1.16                    | 置信度高
	     j4 | 未测 → 保持模型                                    | —
	     j5 | 实测 0.011（模型 0.0091）→ 1.20                    | 置信度高
	     j6 | 实测 ~0（模型 0）→ 保持模型：惯性项只占扭矩 2~13%，不可辨识，但结果与模型一致
	     j7 | 未测 → 保持模型                                    | —

	   尚存的不确定度（不影响当前使用，但换硬件时要重测）：
	     · j1 的 kJ_offset 可能含“测试时末端挂的载荷”，换工具后需重测；
	     · j5 实测随 j6 从 0.011 变到 0.005，但折合力矩仅 ~0.06 N·m（与摩擦同量级），模型
	       M55 恒为 0.0091 表达不出来；等法兰上装真实工具再填 kMass[6]/kLength[6]；
	     · j1 偏小 3~4 倍的原因未定论，最可能是 mdh_left 的 a/d 全为 0（缺基座-肩偏置）。 */
	inline constexpr float kJ_scale[Joint_N::dof]  = {3.30f, 1.00f, 1.16f, 1.00f, 1.20f, 1.00f, 1.00f};
	inline constexpr float kJ_offset[Joint_N::dof] = {0.109f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f};
}

class Joint : public Motor_DM
{
	public:
	explicit Joint(uint8_t host_id_, uint8_t motor_id_, const Motor_DM_N::param_factor_s& facs_,
		Bsp_CAN& can_, float target_pos_, const Joint_N::angle_limit_s& angle_limit_,
		float offset_, const RoboArm_N::mdh_param_s& mdh_,
		const pid_pos_cfg_s& pid_cfg_, uint16_t kp_pos_, uint16_t kp_vel_,
		float kq_vel_, float remote_vel_max_)
		: Motor_DM(host_id_, motor_id_, facs_, can_), angle{}, tg(0.f), td(0.f), target_pos(target_pos_),
		angle_limit(angle_limit_), offset(offset_), mdh(mdh_), pos_wrapped_last(0.f), pos_unwrapped(0.f),
		pid(pid_cfg_), kp_pos(kp_pos_), kp_vel(kp_vel_), kq_vel(kq_vel_), remote_vel_max(remote_vel_max_),
		planner{}, pos_ref(0.f), vel_ref(0.f), acc_ref(0.f), target_dirty(false), inited(false), plan_inited(false)
	{}

	/* ---- 反馈入口：CAN 接收回调里调用 ---- */
	void UpdateFeedback(const uint8_t* data_);

	/* ---- 每周期先更新全部关节参考，再算动力学前馈，最后分组 ControlStep ---- */
	void UpdateReference();
	void ControlStep();

	/* ---- 目标入口 ---- */
	void SetTarget(float pos_);      // 绝对位置入口 (rad)：一段规划到点停车（终点速度 0）
	void SetStreamTarget(float pos_); // 流式目标入口 (rad)：终点速度取目标自身速度，可被连续刷新
	void SetVelocity(float vel_);    // 速度入口 (rad/s)，仅在控制任务调用；零速度减速保持
	void SetRemoteVelocity(float ratio_, bool sticks_neutral_ = true); // 零输入不抢占位置模式，恢复须真实回中

	/* ---- 生命周期：解挂起/使能前调用（详见 Joint_SoftStartAll） ---- */
	void CaptureHoldPosition();      // 控制任务挂起后、失能前，保存急停恢复目标
	void SoftStart();

	Joint_N::angle_s angle;                                    // 角度信息：theta + base
	float tg;                                                  // 重力前馈力矩 (Nm)，由 RoboArmKine::Solve() 写入
	float td;                                                  // 惯性/科氏前馈力矩 (Nm)，由 RoboArmKine::Solve() 写入
	const RoboArm_N::mdh_param_s& Mdh() const { return mdh; }  // 改进 DH 参数

	/* 规划器参考量（供运动学前馈与调试观测；只读） */
	float RefPos() const { return pos_ref; }
	float RefVel() const { return vel_ref; }
	float RefAcc() const { return acc_ref; }
	bool IsVelocityMode() const { return velocity_mode; }
	float HoldPos() const { return hold_pos; } // 持久保持/复位目标，保护裁剪不能覆盖它

	/* 实际连续角（编码器尺度）：上位机协议【收发双向】都用这一个约定，
	   也是 SetTarget / SetStreamTarget 入参的约定。由电机单圈角展开而来，可以超出 ±π。 */
	float PosUnwrapped() const { return pos_unwrapped; }
	/* 关节角 θ（MDH 约定）= mdh.theta_offset + 编码器连续角 - offset。
	   这是运动学模型（roboarm_kinematics）专用的量，与 PosUnwrapped() 恒差一个
	   各轴不同的常量，**不用于上位机协议**。 */
	float Theta() const { return angle.theta; }
	bool IsInited() const { return inited; }

	private:
	void ApplyLimit();    // 把 target_pos 钳到（圈次对齐后的）关节限位内
	void PlanToTarget();  // 以当前参考状态为起点，规划一段三次多项式到 target_pos（终点速度 0）
	void PlanToStreamTarget(float feedback_pos_); // 同上，但终点速度取目标速度估计（可以为非 0）
	void UpdateStreamVelocity(bool new_target_);  // 估计“目标自己跑多快”，作为流式段的终点速度
	void UpdatePositionReference(float feedback_pos_); // 按跟踪偏差平滑调节轨迹时钟，同步换算 v/a
	void UpdateLimitState(float feedback_pos_); // 实际位置方向保护，带退出滞回
	void GuardReference(float feedback_pos_);   // 参考硬限位 + 清除已触限方向的运动欠账
	void SyncReference();                      // 裁剪后同步规划器，防止下一拍重放旧状态
	void RebaseReference(float feedback_pos_);  // 只从反馈重建规划起点，保留保持目标

	float target_pos;                 // 当前可执行段的目标；与持久 hold_pos 分离
	const Joint_N::angle_limit_s angle_limit;
	const float offset;               // 编码器零点偏置
	const RoboArm_N::mdh_param_s mdh;
	float pos_wrapped_last;           // 上一帧单圈角（用于展开连续角）
	float pos_unwrapped;              // 编码器连续角（控制用的实际位置）
	pid_pos_s pid;                    // 位置环 PID（输出纠偏速度，参数在 pid.hpp 的 RoboArm_Pid）
	const uint16_t kp_pos;            // MIT kp 字段（不把位置误差交给电机时填 0）
	const uint16_t kp_vel;            // MIT kd 字段（电机速度阻尼）
	const float kq_vel;               // 速度误差→力矩补偿增益 (Nm·s/rad)，叠加到 torque_ff
	                                  // 补 MIT kd 的权限：顶满 kd 也追不上 v_cmd 的关节（如 j2）用，
	                                  // 0 表示不补偿。取值为 0 时行为与不接该通路一致。
	const float remote_vel_max;       // 满量程遥控速度 (rad/s)，与任务周期无关
	Route_Plan_N::cubic_plan_s planner;  // 三次多项式位置规划器（替代原斜坡）
	float pos_ref;                    // 规划输出位置（控制参考）
	float vel_ref;                    // 规划输出速度（前馈速度）
	float acc_ref;                    // 规划输出加速度（惯性前馈用）
	bool target_dirty;                // 位置模式需要重规划；仅控制任务读写
	bool inited;                      // 是否已收到有效反馈
	bool plan_inited;                 // 规划是否已用实测位置初始化（缓启动标志）
	Route_Plan_N::velocity_plan_s velocity_planner{};
	float velocity_cmd{0.f};
	bool velocity_mode{true};         // 速度模式；松杆停稳后自动切入固定位置保持
	bool remote_active{false};
	float limit_min{0.f};             // 第一帧反馈确定圈次，运行中不随目标跨圈漂移
	float limit_max{0.f};
	float position_request{0.f};
	volatile bool position_pending{false}; // 与 SetTarget / SetStreamTarget 的请求值用短临界区成对读写
	int limit_side{0};                // -1 下限，+1 上限；按实际反馈判定，输出端也复核
	float hold_pos{0.f};              // 只随用户目标/实际可执行遥控轨迹更新，保持时不随反馈漂移
	bool hold_valid{false};
	bool restoring_position{false};  // 自动保持/复位，区别于尚未到达的显式位置命令
	bool hold_rebase_armed{false};    // 实际回到保持点后才允许下一次外力偏移重建起点
	float position_time_scale{1.f};  // d(规划时间)/d(真实时间)，跟不上时平滑降到零
	float position_scale_rate{0.f};  // 时间倍率变化率上限，由轨迹峰值速度和剩余加速度预算确定
	/* ---- 流式目标状态（只有控制任务读写；请求标志除外）---- */
	volatile bool stream_request{false}; // 与 position_request 同一临界区发布：本次请求是流式目标
	bool stream_mode{false};         // 本关节当前走流式规划；绝对位置/遥控会清掉
	float stream_vel{0.f};           // 目标速度估计 v_d，作为规划段的终点速度
	float stream_vel_goal{0.f};      // v_d 的收敛目标：目标跳变时取差商，停流后置 0
	float stream_target_prev{0.f};   // 上一次上游目标，用来算差商
	uint16_t stream_idle_cycles{0U}; // 目标连续不变的周期数，用来判“上游停住”
	bool remote_wait_neutral{false};  // 急停恢复后须先回中，旧杆量不能覆盖保存位置
	float resume_pos{0.f};            // 恢复快照，防止挂起发生在控制函数中间时被旧周期覆盖
	bool resume_valid{false};
	volatile uint32_t reset_generation{0U};
};

/* 全部关节缓启动：由 Error_tasks 在解挂起/使能之前调用。
   必须在执行任务（roboARM）处于挂起状态时调用，否则会与规划器的采样打架。 */
void Joint_SoftStartAll();
void Joint_CaptureHoldAll();       // 挂起执行任务后、失能前调用，整个挂起期间只保存一次


/* ============================== 夹爪（第 7 轴）==============================
   j7 装上夹爪后不再走 Joint 的规划/PID 通路，改成「两点位置 + 驱动器自带刚度」：
     · 只有两个预设位（张开 / 抓紧），由遥控 ch3 的正负向或上位机的夹爪命令字节切换；
     · 位置伺服整个交给驱动器的 MIT 环路：固件只发 pos_des 与固定的 kp/kd，
       vel_des 与 tq_ff 恒为 0 —— 等价于在达妙调参软件里发一条位置指令；
     · 不走 Joint::ControlStep() 的三个理由：
         ① 那里用 WrapRad(pos_ref - pos_unwrapped) 取最短角，两个预设位相距 9 rad，
            会被折成反向的 2.7 rad，夹爪会往反方向跑；
         ② ApplyLimit() 的 ±3 窗口（跨度 6 rad）装不下相距 9 rad 的两个点；
         ③ MIT kp 的越差保护（误差 > mit_kp_err_max 就把 kp 发 0）会恰好在最需要推力时清零。
   ⭐ 夹爪的全部参数（pos_max / 两个预设位 / kp / kd）与其它关节一样统一登记在
   variables.cpp，本文件不再重复定义；这里只声明两个只读访问器供协议层刷调试镜像。 */

/* 夹爪两个预设位（pos_des，rad，与达妙上位机显示的位置同一把尺子），定义在 variables.cpp */
float Gripper_OpenPosition(void);    // 张开位
float Gripper_ClosePosition(void);   // 夹紧位

/* 控制周期调用（替代原来的 j7_left.ControlStep()）：持续下发夹爪的 MIT 位置指令。 */
void Gripper_Update(void);
