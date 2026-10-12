#include "threads.hpp"
#include "roboarm.hpp"
#include "roboarm_kinematics.hpp"
#include "Remote.h"


extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;
extern Joint j7_left;

extern RoboArmKine robo_arm_kine;


/* 调试观测：计算/调度错过 2ms 截止时间的次数。超期不补跑历史积分。 */
volatile uint32_t arm_control_overrun_count = 0;

/* 全轴先生成同一拍参考，再算前馈，CAN 仍按四轴/三轴分组发送。
   固定 2ms 逻辑时基对应规划器与 PID；绝对时间调度避免相对延时累计漂移。
   挂起后只重建一次参考，不按挂起时长补积分。
   j7 是夹爪：不参与参考/前馈，第二组里改调 Gripper_Update() 直接下发驱动器。 */
void roboARM_task(void *argument)
{
	UNUSED(argument);
	const uint32_t cycle_ticks = osKernelGetTickFreq() / 500U;
	uint32_t cycle_start = osKernelGetTickCount();

	for (;;)
	{
		// 挂起/超期后丢弃旧截止时间，不连续补发追赶过去的周期。
		const uint32_t now = osKernelGetTickCount();
		if (static_cast<int32_t>(now - cycle_start) >= static_cast<int32_t>(cycle_ticks))
		{
			++arm_control_overrun_count;
			cycle_start = now;
		}
		Remote_UpdateJointVelocity();
		j1_left.UpdateReference();
		j2_left.UpdateReference();
		j3_left.UpdateReference();
		j4_left.UpdateReference();
		j5_left.UpdateReference();
		j6_left.UpdateReference();
		/* j7（夹爪）不跑参考：它的输出是 Gripper_Update()，不受规划器/PID 影响。
		   angle.theta / base 照旧由 CAN 反馈与 robo_arm_kine.Solve() 维护，不受影响。 */
		robo_arm_kine.Solve();  // 本拍 RefVel/RefAcc → 重力/惯性前馈
		j1_left.ControlStep();
		j2_left.ControlStep();
		j3_left.ControlStep();
		j4_left.ControlStep();
		const uint32_t second_group = cycle_start + cycle_ticks / 2U;
		if (static_cast<int32_t>(second_group - osKernelGetTickCount()) > 0) osDelayUntil(second_group);
		j5_left.ControlStep();
		j6_left.ControlStep();
		Gripper_Update();  // j7：两点位置 + 驱动器自带 kp/kd，见 roboarm.hpp 顶部的夹爪说明
		cycle_start += cycle_ticks;
		const int32_t remaining = static_cast<int32_t>(cycle_start - osKernelGetTickCount());
		if (remaining > 0) osDelayUntil(cycle_start);
		else if (remaining < 0)
		{
			++arm_control_overrun_count;
			cycle_start = osKernelGetTickCount();
			osDelay(1); // 超期时仍让出 CPU，生命周期任务可以执行挂起
		}
	}
}
