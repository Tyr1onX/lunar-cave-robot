#include "bsp_tim.hpp"
#include "mathetics.hpp"
#include "pid.hpp"
#include "roboarm.hpp"
#include "roboarm_kinematics.hpp"
#include "Host.h"
#include "ring_buffer.hpp"

/* 
          上电机械零位      上电关节角
j1 8009p  -0.21044         
j2 8009p -2.38221
j3 4340p 0.99863
j4 4310P -2.80770
J5 4310P 0.58819
J6 4310 -1.448012
J7 3507 -1.28081
*/
/* place all variables in one file to avoid initialization order issues */

Timebase_Timer micro_base;

/* Match MX_FDCAN3_Init(): standard-ID classic CAN frames for DAMIAO motors. */
Bsp_CAN abc_can3(&hfdcan3, false, true, true);

/* ==================== 电机本体参数（唯一登记处）====================
   这里登记的是「电机在达妙调参软件里的配置」，不是控制层参数。改软件必须回来改这里：
   MakeFactors(pos, vel, torque) 的三个量程要与软件里对应电机的量程逐项一致，
   否则固件报出的角度/速度/力矩、以及发出去的指令都会整体差一个比例。

   ⚠️ 位置量程 = **位置字段的半量程**（位置字段 = ±量程）。达妙软件里那个 pos_max 框
   到底是「全量程」还是「半量程」，按下面这个现象对：把它设成 X，看软件能不能把
   电机开到 X；开不到、只能开到 X/2 → 那个框是全量程，填进 MakeFactors 的要除 2。
   ⚠️ 这个值同时是「固件 rad ↔ 物理弧度」的比例：同一串指令在量程翻倍后对应的物理
   转角也翻倍。所以改它会影响关节限位 angle_limit、remote_vel_max、PID 的
   vel_max/accel_max、辨识出的 kJ_*、以及上位机的竖直偏置表。
   ⚠️ **只改你在软件里确实动过的那个电机**，其余保持原值。 */
constexpr float arm_pos_max = 6.28318f;      // j1..j6 的电机：达妙软件里没有动过
constexpr float gripper_pos_max = 12.556f;   // j7 夹爪电机：达妙上位机里 pos_max 框的值

/* ---- 夹爪（j7）控制参数：与其它关节一样统一在这里登记，别处不要再写一份 ----
   两个预设位就是 pos_des 本身（rad，与达妙上位机显示的位置同一把尺子），
   必须落在 ±gripper_pos_max 内，否则会被 Gripper_Update() 钳住、到不了那个位置。 */
constexpr float gripper_open_pos = 12.f;      // 张开位
constexpr float gripper_close_pos = 0.f;    // 夹紧位
/* ---- 夹爪 MIT kp/kd ----
   ⚠️ 口径：0.8 / 0.03 是**电机端实际生效的物理增益**（也就是 12 位字段映射之后的结果），
   不是报文里那个原始整数。报文 kp 字段是 12 位整数（0~4095 ↔ 0~500 Nm/rad，
   LSB ≈ 0.122 Nm/rad），0.8 只能取最接近的 **7** → 实际生效 7×500/4095 ≈ 0.855 Nm/rad；
   kd 字段 0~4095 ↔ 0~5 Nm·s/rad，0.03 → **25**（实际 0.0305）。
   ⇒ 想让电机端就是某个物理增益，改上面的 gripper_kp/gripper_kd；
     想直接指定报文里的整数，改下面的 xxx_field。 */
constexpr float gripper_kp = 0.8f;           // 电机端生效的 MIT kp (Nm/rad)
constexpr float gripper_kd = 0.03f;          // 电机端生效的 MIT kd (Nm·s/rad)
constexpr uint16_t gripper_kp_field =
	static_cast<uint16_t>(gripper_kp * (4095.0f / 500.0f) + 0.5f);   // 报文值 = 7
constexpr uint16_t gripper_kd_field =
	static_cast<uint16_t>(gripper_kd * (4095.0f / 5.0f) + 0.5f);     // 报文值 = 25

/* 各型号的速度/力矩量程同样取自达妙手册与调参软件 */
constexpr Motor_DM_N::param_factor_s dm3507 = Motor_DM_N::MakeFactors(gripper_pos_max, 50.f, 5.f);
constexpr Motor_DM_N::param_factor_s dm4340p = Motor_DM_N::MakeFactors(arm_pos_max, 10.f, 28.f);
constexpr Motor_DM_N::param_factor_s dm4310 = Motor_DM_N::MakeFactors(arm_pos_max, 30.f, 10.f);
constexpr Motor_DM_N::param_factor_s dm4310p = Motor_DM_N::MakeFactors(arm_pos_max, 30.f, 10.f);
constexpr Motor_DM_N::param_factor_s dm8009p = Motor_DM_N::MakeFactors(arm_pos_max, 45.f, 54.f);

/* 满量程遥控速度 (rad/s)，由旧步长 / 0.01s 换算，保留原有效速度。
   j6/j7 原注释与数值不一致，这里按实际原值 0.012rad/10ms 保留为 1.2rad/s。 */
constexpr float remote_vel_max_j1 = 1.2f;
constexpr float remote_vel_max_j2 = 0.8f;
constexpr float remote_vel_max_j3 = 0.8f;
constexpr float remote_vel_max_j4 = 1.0f;
constexpr float remote_vel_max_j5 = 1.0f;
constexpr float remote_vel_max_j6 = 1.2f;
constexpr float remote_vel_max_j7 = 1.2f; // S1 上位 + S2 下拨时映射到 ch3

/* 构造参数尾部：pid_cfg, kp_pos(MIT kp), kp_vel(MIT kd), kq_vel(速度误差→力矩补偿, Nm·s/rad), remote_vel_max。
   kq_vel 只给"顶满 MIT kd 也追不上 v_cmd"的关节开（当前只有 j2）：它等效提高速度环增益，
   但比电机内部 kd 多一拍 CAN + 一个任务周期的延迟，起点取 2.0 Nm·s/rad（≈ MIT kd 的 40%，
   按 4000 ↔ 4.884 的映射估算），取大同样会振。 */
__attribute__((used)) Joint j1_left(0x11, 0x01, dm8009p, abc_can3,
	0.f, {Math_N::two_pi - 2.8f, -1.7f}, 
	-0.14122, RoboArm_N::mdh_left[0],  
	RoboArm_Pid::j1_left, 600, 4000, 0.f, remote_vel_max_j1);

__attribute__((used)) Joint j2_left(0x12, 0x02, dm8009p, abc_can3,
	-2.8f, {Math_N::two_pi - 2.4f, 1.75f}, 
	-2.38192f, RoboArm_N::mdh_left[1],  
	RoboArm_Pid::j2_left, 500, 4000, 2.f, remote_vel_max_j2);

__attribute__((used)) Joint j3_left(0x13, 0x03, dm4340p, abc_can3,
	-0.0510056764f, {0.99f, -Math_N::pi}, 
	1.00439f, RoboArm_N::mdh_left[2], 
	RoboArm_Pid::j3_left, 0, 4000, 0.f, remote_vel_max_j3);

__attribute__((used)) Joint j4_left(0x14, 0x04, dm4310p, abc_can3,
	-2.80770f, {Math_N::two_pi - 0.15f, 0.8f},
	-2.80760, RoboArm_N::mdh_left[3], 
	RoboArm_Pid::j4_left, 0, 900, 0.f, remote_vel_max_j4);

__attribute__((used)) Joint j5_left(0x15, 0x05, dm4310, abc_can3,
	0.0774672702f, {1.f, -2.6f}, 
	0.62223f, RoboArm_N::mdh_left[4],  
	RoboArm_Pid::j5_left, 200, 1500, 0.f, remote_vel_max_j5);

__attribute__((used)) Joint j6_left(0x16, 0x06, dm4310, abc_can3,
	-0.8f, {1.8f, 2.85f - Math_N::two_pi}, 
	-0.77208, RoboArm_N::mdh_left[5],  
	RoboArm_Pid::j6_left, 0, 500, 0.f, remote_vel_max_j6);

__attribute__((used)) Joint j7_left(0x17, 0x07, dm3507, abc_can3,
	-0.00671127299f, {3.f, -3.f}, 
	3.11051f, RoboArm_N::mdh_left[6],  
	RoboArm_Pid::j7_left, 0, 0, 0.f, remote_vel_max_j7);

/* 7 DAMIAO joint motors on FDCAN3 (motor_id 0x01..0x06, host_id 0x11..0x16).
   Add more Joint instances here (e.g. j7: 0x17/0x07) if the arm grows. */

/* 机械臂运动学：解算连杆单位向量（Joint::angle.base）与重力前馈（Joint::tg） */
RoboArmKine robo_arm_kine({j1_left, j2_left, j3_left, j4_left, j5_left, j6_left, j7_left});

/* 挂起执行任务后、失能前保存一次，挂起期间的反馈变化不能更新恢复目标。 */
void Joint_CaptureHoldAll()
{
	j1_left.CaptureHoldPosition();
	j2_left.CaptureHoldPosition();
	j3_left.CaptureHoldPosition();
	j4_left.CaptureHoldPosition();
	j5_left.CaptureHoldPosition();
	j6_left.CaptureHoldPosition();
	j7_left.CaptureHoldPosition();
}

/* 全部关节缓启动：解挂起/使能之前调用，从当前实测位置规划回保存的位置。
   必须在执行任务挂起期间调用（Error_tasks 负责保证这个顺序）。 */
void Joint_SoftStartAll()
{
	j1_left.SoftStart();
	j2_left.SoftStart();
	j3_left.SoftStart();
	j4_left.SoftStart();
	j5_left.SoftStart();
	j6_left.SoftStart();
	j7_left.SoftStart();
}

/* ============================== 夹爪（j7）==============================
   背景与为什么不走 Joint::ControlStep()，见 roboarm.hpp 顶部的夹爪说明。
   本文件是 j7_left 的所属处，所以控制入口也放这里，与 Joint_*All 同一层。 */

/* 夹爪目标状态（故意不设 static，方便在调试器里直接观察）：1 = 抓紧，0 = 张开。
   摇杆回中/切组时必须保持而不是重算，所以它是一个状态，不是每拍由 ratio 推出的结果。 */
__attribute__((used)) volatile uint8_t gripper_close_target = 0U;

extern "C" void Gripper_Command(const uint8_t close_target_)
{
	gripper_close_target = (close_target_ != 0U) ? 1U : 0U;
}

/* 控制周期调用（500Hz）：持续下发夹爪的 MIT 位置指令。
   驱动器自己闭合位置环，所以固件只给 pos_des 与固定 kp/kd；vel_des 与 tq_ff 恒为 0
   （与在达妙调参软件里发一条位置指令等价）。 */
void Gripper_Update(void)
{
	if (!j7_left.IsInited())
	{
		/* 与 Joint::ControlStep() 的使能时序一致：收到第一帧反馈之前持续发使能 */
		j7_left.Ctrl(Motor_DM_N::order_byte_e::enable);
		return;
	}

	/* 预设位就是 pos_des 本身（rad，与达妙上位机显示的位置同一把尺子），参数在文件顶部登记；
	   再钳到量程内留 1e-3 余量：超出会让 16 位字段绕回，指令会跑到反方向去。 */
	const float preset = (gripper_close_target != 0U) ? gripper_close_pos : gripper_open_pos;
	const float pos_max_abs = gripper_pos_max - 1.e-3f;
	const float pos_des = Limit(preset, pos_max_abs, -pos_max_abs);

	j7_left.Ctrl(pos_des, 0.f, gripper_kp_field, gripper_kd_field, 0.f);
}

/* 两个预设位的只读访问器（协议层刷调试镜像用），取值见本文件顶部的夹爪参数 */
float Gripper_OpenPosition(void) { return gripper_open_pos; }
float Gripper_ClosePosition(void) { return gripper_close_pos; }

/* 供 C 侧（Remote.c）在控制周期内按索引提交归一化速度比例：0..5 -> j1..j6，范围 [-1, 1]。
   Joint 内部按 remote_vel_max 换算 rad/s，UpdateReference 约束速度后积分。
   夹爪 j7 不是速度轴，不走这里（走 Gripper_Command/Gripper_Update）。 */
extern "C" void Joint_SetRemoteVelocity(const uint8_t idx, const float ratio, const uint8_t sticks_neutral)
{
	switch (idx)
	{
	case 0U: j1_left.SetRemoteVelocity(ratio, sticks_neutral != 0U); break;
	case 1U: j2_left.SetRemoteVelocity(ratio, sticks_neutral != 0U); break;
	case 2U: j3_left.SetRemoteVelocity(ratio, sticks_neutral != 0U); break;
	case 3U: j4_left.SetRemoteVelocity(ratio, sticks_neutral != 0U); break;
	case 4U: j5_left.SetRemoteVelocity(ratio, sticks_neutral != 0U); break;
	case 5U: j6_left.SetRemoteVelocity(ratio, sticks_neutral != 0U); break;
	default: break;
	}
}

/* ===== 上位机 USB-CDC 链路（中断 usb_callback.cpp / 任务 task_host.cpp / 协议 app/Host.cpp）=====
   host_usb_queue：USB 中断写、Host_Task 读的无锁 SPSC 环形队列；
   host_link_tick：最近一次收到 USB 数据包的 HAL tick（中断写、Host_IsOnline 读）；
   host_rx_frames：完整合法帧计数（任务写）；host_rx_drop：队列满丢弃的数据包数（中断写）；
   host_tx_drop：上行帧“就绪但还没发出去就被新帧顶掉”的次数（任务写，双缓冲状态机）。
   全部放在本文件，与其它全局对象一样避开初始化顺序问题。 */
Ring_Queue<uint8_t, HOST_RX_QUEUE_SIZE> host_usb_queue;
volatile uint32_t host_link_tick = 0U;
volatile uint32_t host_rx_frames = 0U;
volatile uint32_t host_rx_drop = 0U;
volatile uint32_t host_tx_drop = 0U;

__attribute__((used)) float host_debug_target[HOST_DEBUG_DOF] = {0.f};
__attribute__((used)) uint8_t host_debug_enable = 0U;
__attribute__((used)) uint8_t host_debug_gripper = 0U;
__attribute__((used)) uint8_t host_debug_adopted = 0U;

