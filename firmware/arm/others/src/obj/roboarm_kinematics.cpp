#include "roboarm_kinematics.hpp"
#include <cmath>

namespace
{
struct V3 { float x, y, z; };
V3 cross(V3 a,V3 b)
	{return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
float dot(V3 a,V3 b)
	{return a.x*b.x+a.y*b.y+a.z*b.z;}
V3 add(V3 a,V3 b)
	{return {a.x+b.x,a.y+b.y,a.z+b.z};}
V3 sub(V3 a,V3 b)
	{return {a.x-b.x,a.y-b.y,a.z-b.z};}
V3 scale(V3 a, float s)
	{return {a.x*s,a.y*s,a.z*s};}

/* 粗略杆件模型（kg, m）。索引 i = “位于关节 i+1 之后的那根杆”，即 link i 从关节 i+1 的原点
   沿 angle.base[i] 伸出 kLength[i]，质心假设在杆中心（0.5·L 处）：
     i=0 : 底座回转（j1）自己的部分，通常为 0
     i=1 : 肩杆（j2 驱动）     i=2 : 肘杆（j3 驱动）
     i=3..5 : 前臂/腕杆        i=6 : **末端（j7 之后）的杆/工具**
   注意：**i=6 就是机械臂的末端（从 j7 往后的法兰/工具）**。现在是 0，
   所以 j6/j7 的重力前馈恒为 0（实测已验证）；j7 法兰上装了工具或负载时，
   必须把质量/长度填在 kMass[6]/kLength[6]，否则 j7 会因没有前馈而掉落。
   这不是惯量模型：只用于重力前馈，转动惯量、摩擦、工具几何都还没建模。 */
constexpr float kMass[Joint_N::dof] = {0.f,1.2f,1.5f,0.f,0.8f,0.f,0.f};
constexpr float kLength[Joint_N::dof] = {0.f,0.320f,0.325f,0.f,0.213f,0.f,0.f};

/* 杆链几何 + 关节转轴（均已在基座系下）：重力与惯性前馈共用同一套模型，每周期只算一次 */
struct chain_s
{
	V3 joint_pos[Joint_N::dof + 1];  // 关节 i 原点；[dof] = 链末端（j7 之后的法兰点）
	V3 com[Joint_N::dof];            // 杆 i 的质心
	V3 axis[Joint_N::dof];           // 关节 i 的转轴（施加 Rot_X(alpha_i) 后 frame 的 z 轴）
};

using JointRefs = std::array<std::reference_wrapper<Joint>, Joint_N::dof>;

void BuildChain(const JointRefs& joints, chain_s& c)
{
	// 1) 位置链：link i 从关节 i 原点沿 angle.base[i] 伸出 kLength[i]，质心在杆中心
	V3 origin{0.f, 0.f, 0.f};
	for (uint8_t i = 0; i < Joint_N::dof; ++i)
	{
		c.joint_pos[i] = origin;
		const float* u = joints[i].get().angle.base;
		c.com[i] = {origin.x + 0.5f * kLength[i] * u[0],
		            origin.y + 0.5f * kLength[i] * u[1],
		            origin.z + 0.5f * kLength[i] * u[2]};
		origin = {origin.x + kLength[i] * u[0],
		          origin.y + kLength[i] * u[1],
		          origin.z + kLength[i] * u[2]};
	}
	c.joint_pos[Joint_N::dof] = origin;

	// 2) 转轴：累乘 R = R·Rot_X(alpha_i)·Rot_Z(theta_i) 后取 frame 的 z 轴
	float rx[3] = {1.f, 0.f, 0.f};
	float ry[3] = {0.f, 1.f, 0.f};
	float rz[3] = {0.f, 0.f, 1.f};
	for (uint8_t i = 0; i < Joint_N::dof; ++i)
	{
		const Joint& j = joints[i].get();
		const float theta = j.angle.theta;
		const float alpha = j.Mdh().alpha;
		const float ct = cosf(theta), st = sinf(theta);
		const float ca = cosf(alpha), sa = sinf(alpha);

		float nx[3], ny[3], nz[3];
		for (uint8_t k = 0; k < 3; ++k)
		{
			nx[k] = ct * rx[k] + st * ca * ry[k] + st * sa * rz[k];
			ny[k] = -st * rx[k] + ct * ca * ry[k] + ct * sa * rz[k];
			nz[k] = -sa * ry[k] + ca * rz[k];
		}
		for (uint8_t k = 0; k < 3; ++k)
		{
			rx[k] = nx[k];
			ry[k] = ny[k];
			rz[k] = nz[k];
		}

		// 注意：必须放在旋转递推之后取轴，否则会丢掉 alpha 的扭转
		c.axis[i] = {rz[0], rz[1], rz[2]};
	}
}

/* 重力前馈 → Joint::tg
   递推方向：**从末端 j7 向基座 j1**。递推量（基座系）：
     f = 已累加的远端重力合力；n = f 对当前关节原点的力矩
   每退一个关节：① 力矩参考点从 joint_pos[i+1] 平移到 joint_pos[i]（力不变，力矩补 r×f）；
                 ② 加入杆 i 自重（作用在 com[i]）；③ tg_i = −n·axis_i */
void AccumulateGravity(const JointRefs& joints, const chain_s& c)
{
	constexpr float g = 9.81f;
	V3 f{0.f, 0.f, 0.f};
	V3 n{0.f, 0.f, 0.f};
	for (int32_t i = static_cast<int32_t>(Joint_N::dof) - 1; i >= 0; --i)
	{
		n = add(n, cross(sub(c.joint_pos[i + 1], c.joint_pos[i]), f));
		const V3 f_i{0.f, 0.f, -kMass[i] * g};
		f = add(f, f_i);
		n = add(n, cross(sub(c.com[i], c.joint_pos[i]), f_i));
		joints[i].get().tg = -dot(c.axis[i], n);
	}
}

/* 惯性/科氏前馈 → Joint::td（点质量模型，暂不计转动惯量 I_C）
   ① 正向遍历（基座 → 末端）求每根杆的 ω、α 与质心加速度（全部基座系）：
        ω_i   = ω_{i-1} + z_i·q̇_i
        α_i   = α_{i-1} + z_i·q̈_i + ω_{i-1} × (z_i·q̇_i)
        a_o,i = a_o,i-1 + α_{i-1} × r_off + ω_{i-1} × (ω_{i-1} × r_off)   （r_off：关节 i-1 → i）
        a_c,i = a_o,i + α_i × r_com + ω_i × (ω_i × r_com)                （r_com：关节 i → 质心）
   ② 达朗贝尔：把惯性力 f_i = −m_i·a_c,i 当作外力，走与重力完全相同的反向递推。
   若以后补 I_C：只需在反向递推里再加一项 N_i = I_c·α_i + ω_i × (I_c·ω_i)。
   q̇/q̈ 用规划器参考量（Joint::RefVel/RefAcc），不是实测差分。
   全轴 UpdateReference() 先于 Solve() 执行，这里读取同一控制周期的参考量。 */
void AccumulateInertia(const JointRefs& joints, const chain_s& c)
{
	V3 f_dyn[Joint_N::dof];  // 每根杆的惯性力
	V3 r_com[Joint_N::dof];  // 关节原点到质心的向量
	V3 omega{0.f, 0.f, 0.f};
	V3 alpha{0.f, 0.f, 0.f};
	V3 a_origin{0.f, 0.f, 0.f};  // 关节原点的加速度

	for (uint8_t i = 0; i < Joint_N::dof; ++i)
	{
		const V3 z = c.axis[i];
		const V3 r_off = (i == 0) ? V3{0.f, 0.f, 0.f} : sub(c.joint_pos[i], c.joint_pos[i - 1]);
		r_com[i] = sub(c.com[i], c.joint_pos[i]);

		// 先把参考点从关节 i-1 推到关节 i（用上游杆 i-1 的 ω/α；i=0 时两者均为 0）
		a_origin = add(a_origin, add(cross(alpha, r_off), cross(omega, cross(omega, r_off))));

		// 再施加关节 i 的转动，得到杆 i 的 ω/α
		const V3 omega_prev = omega;
		const V3 z_qd = scale(z, joints[i].get().RefVel());
		omega = add(omega_prev, z_qd);
		alpha = add(add(alpha, scale(z, joints[i].get().RefAcc())), cross(omega_prev, z_qd));

		// 质心加速度 → 惯性力（达朗贝尔）
		const V3 a_c = add(a_origin, add(cross(alpha, r_com[i]), cross(omega, cross(omega, r_com[i]))));
		f_dyn[i] = scale(a_c, -kMass[i]);
	}

	// 反向递推：与重力前馈同一结构，外力换成惯性力
	V3 f{0.f, 0.f, 0.f};
	V3 n{0.f, 0.f, 0.f};
	for (int32_t i = static_cast<int32_t>(Joint_N::dof) - 1; i >= 0; --i)
	{
		n = add(n, cross(sub(c.joint_pos[i + 1], c.joint_pos[i]), f));
		f = add(f, f_dyn[i]);
		n = add(n, cross(r_com[i], f_dyn[i]));
		joints[i].get().td = -dot(c.axis[i], n);
	}

	/* 对角项用实测值替掉模型值（见 RoboArm_N::kJ_scale / kJ_offset）。
	   点质量模型的 M_ii = Σ_{k>=i} m_k·d_k²，d_k 是杆 k 质心到关节 i 轴线的距离，
	   与上面递推得到的对角项完全等价，所以把差值直接补到 td 上：
	     td_i = Σ_k M_ik·q̈_k  →  J_i·q̈_i + Σ_{k≠i} M_ik·q̈_k  ，J_i = scale_i·M_ii + offset_i
	   这样非对角/科氏项仍然来自模型，只有对角惯量是实测的。 */
	for (uint8_t i = 0; i < Joint_N::dof; ++i)
	{
		const V3 z = c.axis[i];
		float m_ii = 0.f;
		for (uint8_t k = i; k < Joint_N::dof; ++k)
		{
			const V3 r = sub(c.com[k], c.joint_pos[i]);
			const float r2 = dot(r, r);
			const float rz = dot(r, z);
			m_ii += kMass[k] * (r2 - rz * rz);   /* d² = |r|² - (r·z)² */
		}
		const float j_used = RoboArm_N::kJ_scale[i] * m_ii + RoboArm_N::kJ_offset[i];
		joints[i].get().td += (j_used - m_ii) * joints[i].get().RefAcc();
	}
}
}


/* 每周期调用：解算连杆单位向量 → Joint::angle.base，再算重力/惯性前馈 → Joint::tg / td */
void RoboArmKine::Solve() const
{
	// 基座坐标系旋转矩阵 R 的三列，初始为单位阵（R_0^0）
	float rx[3] = {1.f, 0.f, 0.f};
	float ry[3] = {0.f, 1.f, 0.f};
	float rz[3] = {0.f, 0.f, 1.f};

	// 从底部向末端逐关节解算。joints[i] 对应连杆 i+1，其局部系为 frame_{i+1}。
	// 先累乘当前关节的旋转 R = R * Rot_X(alpha_i) * Rot_Z(theta_{i+1}) 得到
	// frame_{i+1} 相对基座的旋转，再把连杆局部方向向量 mdh.axis 变换到基座系：
	// g = R * axis，即连杆单位向量在基座坐标系下的表示。
	for (uint8_t i = 0; i < Joint_N::dof; ++i)
	{
		const Joint& j = joints[i].get();

		// 累乘旋转矩阵：R = R * Rot_X(alpha_i) * Rot_Z(theta_{i+1})
		const float theta = j.angle.theta;
		const float alpha = j.Mdh().alpha;
		const float ct = cosf(theta), st = sinf(theta);
		const float ca = cosf(alpha), sa = sinf(alpha);

		// Rot_X(alpha) * Rot_Z(theta) 的三列：
		// col0 = (ct, st*ca, st*sa), col1 = (-st, ct*ca, ct*sa), col2 = (0, -sa, ca)
		float nx[3], ny[3], nz[3];
		for (uint8_t k = 0; k < 3; ++k)
		{
			nx[k] = ct * rx[k] + st * ca * ry[k] + st * sa * rz[k];
			ny[k] = -st * rx[k] + ct * ca * ry[k] + ct * sa * rz[k];
			nz[k] = -sa * ry[k] + ca * rz[k];
		}
		for (uint8_t k = 0; k < 3; ++k)
		{
			rx[k] = nx[k];
			ry[k] = ny[k];
			rz[k] = nz[k];
		}

		// 连杆方向向量在基座系下的坐标 g = R * axis（axis 为单位向量，R 为旋转矩阵，
		// 理论上 g 已为单位向量；浮点误差累积下再做一次归一化保证模长为 1）
		const float* axis = j.Mdh().axis;
		float gv[3];
		for (uint8_t k = 0; k < 3; ++k)
		{
			gv[k] = rx[k] * axis[0] + ry[k] * axis[1] + rz[k] * axis[2];
		}
		const float norm = sqrtf(gv[0] * gv[0] + gv[1] * gv[1] + gv[2] * gv[2]);
		const float inv = norm > 0.f ? 1.f / norm : 1.f;

		float* base = joints[i].get().angle.base;
		for (uint8_t k = 0; k < 3; ++k)
		{
			base[k] = gv[k] * inv;
		}
	}

	// 杆链几何/转轴每周期只算一次，重力与惯性两个前馈共用
	chain_s c;
	BuildChain(joints, c);
	AccumulateGravity(joints, c);   // → Joint::tg（重力前馈）
	AccumulateInertia(joints, c);   // → Joint::td（惯性/科氏前馈）
}

/* ==================== 预留接口（存根，未实现） ====================
 * 正解/反解都还没有实现，一律返回 false，且不修改传入的输出。
 * 补实现前必须先解决：
 *   1) RoboArm_N::mdh_left 的 a/d 目前全为 0，必须补上真实连杆偏置与长度；
 *      现在 Solve() 只累乘旋转，不是完整正运动学。
 *   2) 递推要用完整变换 T = Rot_X(alpha)·Trans_x(a)·Rot_Z(theta)·Trans_z(d)，
 *      而不是只累乘旋转矩阵。
 *   3) 末端工具坐标（TCP 相对最后一个关节的变换）与关节编号对应关系（当前 mdh_left 有 7 组）。
 *   4) 反解还需要：解支连续性（用 q_seed_ 选解、拒绝跳到另一支）、
 *      关节限位检查（angle_limit）、奇异位形检查。
 *   5) 输出坐标约定：rad，编码器连续角（与 Joint::pos_unwrapped 同系）。
 * ================================================================ */

/* 正解：关节角 → 末端位姿（基座系） */
bool RoboArmKine::GetEndPose(RoboArm_Kine_N::pose_s& pose_) const
{
	(void)pose_;
	return false;
}

/* 反解：末端位姿 → 关节角，q_seed_ 用于选解连续性，q_out_ 长度 = Joint_N::dof */
bool RoboArmKine::SolveInverse(const RoboArm_Kine_N::pose_s& pose_, const float* q_seed_, float* q_out_) const
{
	(void)pose_;
	(void)q_seed_;
	(void)q_out_;
	return false;
}


