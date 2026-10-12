#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include "roboarm.hpp"


/* ============ 机械臂运动学：连杆方向 + 重力/惯性前馈（+ 预留 FK/IK）============
 * 已实现（Solve，每周期调用一次）：
 *   1) 由各关节编码角度，从基座底部向末端逐关节累乘旋转矩阵，把每根连杆在其局部
 *      坐标系中的方向向量（mdh.axis）变换到基座坐标系，写入 Joint::angle.base[3]；
 *   2) 从末端（j7）向基座（j1）递推力旋量，算出两个独立的前馈力矩：
 *        Joint::tg — 重力前馈（外力 = 自重）
 *        Joint::td — 惯性/科氏前馈（外力 = −m·a_c，点质量模型；q̇/q̈ 取规划器参考量）
 *      两个字段分开存放，便于单独启用、单独观测与对比。
 *   offset / alpha / theta_offset / axis 统一存放在 Joint 中，此处直接读取，
 *   不涉及平移 a/d、雅可比等传统动力学矩阵（杆链由 kMass/kLength 近似拼出）。
 *
 * 预留（存根，未实现）：GetEndPose() 正解、SolveInverse() 反解。
 *   实现前需要补齐的东西见 roboarm_kinematics.cpp 的说明。
 * ====================================================================== */

namespace RoboArm_Kine_N
{
	/* 末端位姿（基座坐标系）：位置 (m) + 旋转矩阵（行主序 rot[行][列]） */
	struct pose_s
	{
		float pos[3];      // x, y, z
		float rot[3][3];   // R
	};
}

class RoboArmKine
{
	public:
	explicit RoboArmKine(const std::array<std::reference_wrapper<Joint>, Joint_N::dof>& joints_)
		: joints(joints_)
	{}

	/* 每周期调用：解算连杆方向 → Joint::angle.base，算重力前馈 → Joint::tg，算惯性前馈 → Joint::td */
	void Solve() const;

	/* ---------------- 预留接口（存根：返回 false 且不修改输出） ---------------- */

	/* 正解：由当前关节角求末端位姿（基座系） */
	bool GetEndPose(RoboArm_Kine_N::pose_s& pose_) const;

	/* 反解：由目标末端位姿求关节角。
	   q_seed_ 为初值（建议用上一解，保证解支连续）；q_out_ 长度 = Joint_N::dof，
	   单位 rad，坐标与 Joint::pos_unwrapped（编码器连续角）一致。 */
	bool SolveInverse(const RoboArm_Kine_N::pose_s& pose_, const float* q_seed_, float* q_out_) const;

	private:
	const std::array<std::reference_wrapper<Joint>, Joint_N::dof> joints;
};
