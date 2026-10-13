#include "Host.h"

#include "Remote.h"      /* Remote_IsHostControl()：S2 上拨才采纳位置目标 */
#include "roboarm.hpp"
#include "ring_buffer.hpp"

#include "usbd_cdc_if.h"      /* 自带 extern "C" 保护，放在这里即可 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* USB 设备句柄（定义在 USB_DEVICE/App/usb_device.c） */
extern USBD_HandleTypeDef hUsbDeviceHS;

#ifdef __cplusplus
}
#endif

#include <cmath>
#include <cstring>

/* 关节对象定义在 variables.cpp */
extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;
extern Joint j7_left;

/* ============================ 上位机 USB-CDC 协议 ============================
   本文件只负责协议本身：帧同步、校验、拆包、执行目标、组包与发送。
   任务体在 others/src/thread/task_host.cpp（调 Host_Poll()），
   中断钩子在 others/src/callcallback/usb_callback.cpp（调 Host_UsbReceive()）。
   收发包格式与参考工程 task_host.cpp 完全一致（下行 46B / 上行 35B）；
   本机是纯机械臂：帧里的底盘字段保留位置、只解析不使用，第 7 轴（j7）用夹爪命令字节表达。
   下行（上位机 -> 本机）定长 46B：
     [0]      0xA5  帧头
     [1..12]  3 × float32  底盘速度 (vx, vy, wz)：本机无底盘，解析后丢弃（占位）
     [13]     0x4A  ┐同步字（与帧头、帧尾一起做三重确认）
     [14]     0xB5  ┘
     [15]     使能字节：非 0 才采纳本帧的关节目标与夹爪命令（0 = 只回状态，不改目标）
     [16..19] 4 字节预留（参考工程此位置未使用），本机忽略
     [20..43] 6 × float32  目标编码器连续角 (rad)  → j1..j6，与 Joint::SetStreamTarget 同一约定
     [44]     夹爪命令（**有符号**）：+1 = 抓紧（j7 → 夹爪夹紧位）、
              -1 = 张开（→ 夹爪张开位）、0 = 保持上一状态
     [45]     0x4B  帧尾
   上行（本机 -> 上位机）定长 35B：
     [0..3]   0x5A 0x01 0x4A 0xB5
     [4]      inited：j1..j6 是否都已收到有效反馈
     [5..8]   count：uint32 小端递增序号，上位机据此判断丢包
     [9..32]  6 × float32  实际编码器连续角 (rad)（j1..j6，Joint::PosUnwrapped）
     [33]     夹爪当前命令（与下行同一约定，有符号）：+1 = 抓紧，-1 = 张开
     [34]     0x4B  帧尾

   角度约定（⭐ 收发双向都是【编码器弧度】这唯一一把尺子，固件不做任何换算）：
     上行 = Joint::PosUnwrapped()，下行目标回发同一量；上位机自己持有「整臂完全竖直」
     姿态下各轴的编码器值 enc_vertical[i] 作为偏置：
       发目标：target_enc = q_des + enc_vertical[i]   （q_des = 相对竖直的关节角）
       收反馈：q_meas     = enc_recv - enc_vertical[i]
     ⚠️ 偏置表必须取自本协议的上行字段（pos_unwrapped），不能取电机手册的单圈读数，
        两者可能差 2πk；换了编码器零点/改了 variables.cpp 的 offset 后需重新标定。
     下行目标固件内部会做 UnwrapRadNear(requested, pos_unwrapped) 圈次对齐，
     所以上位机多绕/少绕 2πk 的目标也不会被拒，不需要自己管圈次。

   约定（模式契约）：
     · 只在收到一帧后回一次状态包（请求-应答），不主动周期上报；
     · 目标角经 Joint::SetStreamTarget()（流式入口）下发，规划段的终点速度取目标
       自身的运动速度，匀速跟踪时曲线退化成直线，不做“每帧刹停”；
       不使用 SetVelocity / SetRemoteVelocity，因此不占用、也不修改遥控速度模式；
     · 仅当遥控 S2 拨杆上拨（Remote_IsHostControl()）时才采纳下行目标：
       S2 上拨 = 上位机位置模式（遥控侧提交零速度，已接管的轴减速后保持并让出控制权）；
       S2 中位 = 遥控减速后保持；S2 下拨 = 原遥控速度映射；后两者下目标不生效；
     · j7 按夹爪处理：[44] 是**有符号**命令（+1 抓紧 / -1 张开 / 0 保持），
       只映射到两个预设位（variables.cpp 的夹爪参数）；
       不参与 j1..j6 的目标角字段，也不经过 Joint 的规划/PID 通路。 */

/* 定义在 variables.cpp（全工程变量集中处，避免初始化顺序问题） */
extern Ring_Queue<uint8_t, HOST_RX_QUEUE_SIZE> host_usb_queue;
extern volatile uint32_t host_link_tick;
extern volatile uint32_t host_rx_frames;
extern volatile uint32_t host_rx_drop;
extern volatile uint32_t host_tx_drop;

/* 夹爪当前状态（1 = 抓紧，0 = 张开），定义在 variables.cpp；这里只读，用于调试镜像 */
extern volatile uint8_t gripper_close_target;

namespace
{
	volatile uint32_t host_command_tick = 0U;
	volatile uint8_t host_command_seen = 0U;
	volatile uint8_t host_safety_armed = 0U;
	volatile uint8_t host_disabled_since_reset = 0U;
	volatile uint8_t host_enable_after_disabled = 0U;
	volatile uint8_t host_safety_disable_event = 0U;
	volatile uint8_t host_command_expired = 0U;

	/* ---- 下行帧偏移（46B） ---- */
	constexpr uint16_t rx_chassis = 1U;   /* 3 × float32 底盘速度：解析后丢弃 */
	constexpr uint16_t rx_sync_0 = 13U;
	constexpr uint16_t rx_sync_1 = 14U;
	constexpr uint16_t rx_enable = 15U;
	constexpr uint16_t rx_angles = 20U;   /* j1..j6 目标角 */
	constexpr uint16_t rx_gripper = 44U;  /* j7 夹爪命令 */
	constexpr uint16_t rx_trailer = 45U;

	/* ---- 上行帧偏移（35B） ---- */
	constexpr uint16_t tx_magic = 0U;
	constexpr uint16_t tx_version = 1U;
	constexpr uint16_t tx_sync_0 = 2U;
	constexpr uint16_t tx_sync_1 = 3U;
	constexpr uint16_t tx_inited = 4U;
	constexpr uint16_t tx_count_off = 5U;
	constexpr uint16_t tx_angles = 9U;    /* j1..j6 实际角 */
	constexpr uint16_t tx_gripper = 33U;  /* j7 夹爪当前命令 */
	constexpr uint16_t tx_trailer = 34U;

	constexpr uint8_t magic_byte = 0xA5U;
	constexpr uint8_t tx_magic_byte = 0x5AU;
	constexpr uint8_t sync_byte_0 = 0x4AU;
	constexpr uint8_t sync_byte_1 = 0xB5U;
	constexpr uint8_t trailer_byte = 0x4BU;
	constexpr uint8_t protocol_version = 1U;

	/* 帧里以关节角表达的轴数（j1..j6）；第 7 轴 j7 走夹爪命令字节 */
	constexpr uint8_t host_axis_count = static_cast<uint8_t>(Joint_N::dof - 1U);

	void LatchCommandLeaseExpiry(const uint32_t now)
	{
		if ((host_command_seen != 0U) &&
			((now - host_command_tick) > HOST_COMMAND_LEASE_MS))
		{
			host_command_seen = 0U;
			host_safety_armed = 0U;
			host_command_expired = 1U;
			host_safety_disable_event = 1U;
		}
	}

	bool TargetsAreFinite(const uint8_t* const frame)
	{
		float target[host_axis_count] {};
		std::memcpy(target, &frame[rx_angles], sizeof(target));
		for (uint8_t i = 0U; i < host_axis_count; ++i)
		{
			if (!std::isfinite(target[i])) { return false; }
		}
		return true;
	}

	/* 夹爪命令字节（**有符号**）：> 0 = 抓紧、< 0 = 张开、0 = 保持上一状态。
	   ⚠️ 不能再按「非 0 即抓紧」判断：上位机发 +1/-1 时两个值都非 0，会被判成同一个动作，
	   夹爪将永远打不开。 */
	constexpr int8_t gripper_open_cmd = -1;   /* 约定的“张开”值，用于上行初值/回显 */

	/* 参考工程下行帧里的底盘速度字段：本机只解析、不参与控制 */
	struct chassis_command_s
	{
		float vx;
		float vy;
		float wz;
	};

	/* 任务标志位定义在 Host.h 的 HOST_RX_FLAG，中断侧与任务侧共用 */

	static_assert(host_axis_count == 6U, "上位机协议固定 6 个关节角 + 1 个夹爪命令字节");
	static_assert(HOST_DEBUG_DOF == Joint_N::dof, "调试镜像数组长度必须与关节数一致");
	static_assert(HOST_RX_FRAME_LEN == rx_angles + host_axis_count * sizeof(float) + 2U,
		"下行帧长与关节数不一致");
	static_assert(HOST_TX_RESPONSE_LEN == tx_angles + host_axis_count * sizeof(float) + 2U,
		"上行帧长与关节数不一致");

	/* 关节表：与 variables.cpp 中的定义一一对应（j1..j7）；末位 j7 是夹爪轴 */
	Joint* const joints[Joint_N::dof] =
	{
		&j1_left, &j2_left, &j3_left, &j4_left, &j5_left, &j6_left, &j7_left,
	};

	/* ============================ 上行双缓冲状态机 ============================
	   两份缓冲区各自只有四种状态，写入者（本文件的 Host_Poll 侧）与发送者（USB 栈）分居两端：

	     empty   空：可以被写入者占用
	     writing 写：写入者正在组帧（同一时刻最多一份）
	     ready   就绪：数据完整，等待发送
	     reading 读：已交给 USB 栈（指针在 pClassData->TxBuffer 里），传输完成前不许改写

	   规则：
	     · 写入者只写 empty；没有 empty 时（一份 reading、一份 ready）**丢弃 ready 的那一份**，
	       新帧顶掉没发出去的旧帧，并计入 host_tx_drop —— 保证任何时刻都有可写缓冲，不阻塞解析；
	     · 写完置 ready，随即尝试发送：只有「没有 reading」时才能交接成 reading 并启动传输；
	     · reading → empty 由传输完成中断调 Host_TxComplete() 归还（见 usbd_cdc_if.c 的
	       CDC_TransmitCplt_HS）；若 TxState 已归零而状态还停在 reading
	       （拔线/复位没走完成回调），由 ReclaimCompletedBuffers() 兜底收回；
	     · 序号在交接成功（真的交给 USB）时才提交：丢帧表现为上位机看不到那一帧，
	       不会出现序号重复或跳变。 */
	enum class tx_state_e : uint8_t
	{
		empty = 0U,
		writing = 1U,
		ready = 2U,
		reading = 3U,
	};

	/* 一个上行缓冲区：数据 + 状态 + 本帧携带的序号（序号在交接成功时提交） */
	struct tx_buffer_s
	{
		uint8_t data[HOST_TX_RESPONSE_LEN] {};
		volatile uint8_t state = static_cast<uint8_t>(tx_state_e::empty);
		uint32_t count = 0U;
	};

	tx_buffer_s tx_buffers[2U] {};   /* “两个缓冲区”，每次发送占用其中一份 */

	/* 跨调用保持的协议状态：已交给 USB 的最后一帧序号，以及最近一次夹爪命令（有符号 ±1） */
	uint32_t tx_count = 0U;
	int8_t last_gripper_command = gripper_open_cmd;

	bool HasState(const uint8_t index, const tx_state_e state)
	{
		return tx_buffers[index].state == static_cast<uint8_t>(state);
	}

	void SetState(const uint8_t index, const tx_state_e state)
	{
		tx_buffers[index].state = static_cast<uint8_t>(state);
	}

	/* 上行 inited 只覆盖 j1..j6（与参考工程的 arm.GetJointState() 一致），夹爪轴不参与 */
	bool ArmInited(void)
	{
		for (uint8_t i = 0U; i < host_axis_count; ++i)
		{
			if (!joints[i]->IsInited()) { return false; }
		}
		return true;
	}

	/* 非阻塞启动一次 USB 发送；与 USB 中断抢 TxState，用短临界区保护。
	   返回 false 表示这一拍没发出去（不影响计数，下次收到帧时会重发同样序号）。 */
	bool Transmit(const uint8_t* const data, const uint16_t length)
	{
		bool started = false;
		taskENTER_CRITICAL();
		auto* const cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
		if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && cdc != nullptr && cdc->TxState == 0U)
		{
			if (USBD_CDC_SetTxBuffer(&hUsbDeviceHS,
				const_cast<uint8_t*>(data), length) == USBD_OK)
			{
				started = (USBD_CDC_TransmitPacket(&hUsbDeviceHS) == USBD_OK);
			}
		}
		taskEXIT_CRITICAL();
		return started;
	}

	/* 兜底归还：只要确认不可能有正在传输的包，就把停在 reading 的缓冲收回 empty——
	   · TxState 已归零（拔线/复位没走完成回调）；
	   · 或 pClassData 为空（USB 已被 DeInit/重新枚举中）。
	   没有这一步，拔一次线就会有一份缓冲永久卡在 reading，再也发不出回包。 */
	void ReclaimCompletedBuffers(void)
	{
		const auto* const cdc = static_cast<const USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
		if ((cdc != nullptr) && (cdc->TxState != 0U)) { return; }
		for (uint8_t i = 0U; i < 2U; ++i)
		{
			if (HasState(i, tx_state_e::reading)) { SetState(i, tx_state_e::empty); }
		}
	}

	/* 发送者侧：没有 reading 且存在 ready 时，把 ready 交接给 USB 栈（ready → reading）。
	   交接成功才提交序号；启动失败退回 ready，等下一帧再试。 */
	bool TrySendReady(void)
	{
		bool busy = false;
		uint8_t pending = 2U;   /* 2 = 没有就绪帧 */

		/* “挑一份 ready 并置为 reading”必须与完成中断串行，否则可能出现两份同时 reading */
		taskENTER_CRITICAL();
		for (uint8_t i = 0U; i < 2U; ++i)
		{
			if (HasState(i, tx_state_e::reading)) { busy = true; }
			else if (HasState(i, tx_state_e::ready) && (pending == 2U)) { pending = i; }
		}
		if (!busy && (pending < 2U)) { SetState(pending, tx_state_e::reading); }
		taskEXIT_CRITICAL();
		if (busy || (pending >= 2U)) { return false; }

		if (!Transmit(tx_buffers[pending].data, HOST_TX_RESPONSE_LEN))
		{
			SetState(pending, tx_state_e::ready);
			return false;
		}
		tx_count = tx_buffers[pending].count;
		return true;
	}

	/* 写入者侧：取一份可写缓冲。优先 empty；两份都被占（一份 reading、一份 ready）时
	   丢弃 ready 的那一份（新帧覆盖旧帧）并计数。“选空 / 丢弃”与完成中断串行，
	   避免把刚刚发完的缓冲误当成占用而多丢一帧。 */
	uint8_t AcquireWritableBuffer(void)
	{
		ReclaimCompletedBuffers();

		uint8_t index = 2U;
		taskENTER_CRITICAL();
		for (uint8_t i = 0U; i < 2U; ++i)
		{
			if (HasState(i, tx_state_e::empty)) { index = i; break; }
		}
		if (index >= 2U)
		{
			for (uint8_t i = 0U; i < 2U; ++i)
			{
				if (HasState(i, tx_state_e::ready))
				{
					SetState(i, tx_state_e::empty);
					++host_tx_drop;
					index = i;
					break;
				}
			}
		}
		taskEXIT_CRITICAL();
		return index;   /* 2 = 理论不可达：n = 2 时最多一份 reading */
	}

	/* 组一帧上行状态包并交给发送状态机（序号与夹爪命令用跨调用状态）。 */
	void SendStateResponse(void)
	{
		/* 先把上一帧还没发出去的就绪帧交接掉，保证同时最多一份 ready */
		static_cast<void>(TrySendReady());

		const uint8_t index = AcquireWritableBuffer();
		if (index >= 2U) { return; }

		tx_buffer_s& buffer = tx_buffers[index];
		SetState(index, tx_state_e::writing);

		uint8_t* const response = buffer.data;
		response[tx_magic] = tx_magic_byte;
		response[tx_version] = protocol_version;
		response[tx_sync_0] = sync_byte_0;
		response[tx_sync_1] = sync_byte_1;
		response[tx_inited] = ArmInited() ? 1U : 0U;
		const uint32_t next_count = tx_count + 1U;
		std::memcpy(&response[tx_count_off], &next_count, sizeof(next_count));

		/* 回传编码器连续角 Joint::PosUnwrapped()（rad），**不做任何转换**：
		   收发双向都用同一把尺子（编码器弧度），上位机自己持有「臂完全竖直」姿态下
		   各轴的编码器值作为偏置，用它换算成“相对竖直的关节角”。
		   ⚠️ 偏置表必须取自本字段（pos_unwrapped），不能取电机手册的单圈读数。 */
		for (uint8_t i = 0U; i < host_axis_count; ++i)
		{
			const float angle = joints[i]->PosUnwrapped();
			std::memcpy(&response[tx_angles + static_cast<uint16_t>(i) * sizeof(float)],
				&angle, sizeof(angle));
		}
		response[tx_gripper] = static_cast<uint8_t>(last_gripper_command);
		response[tx_trailer] = trailer_byte;
		buffer.count = next_count;

		SetState(index, tx_state_e::ready);
		static_cast<void>(TrySendReady());
	}

	/* 解析一帧下行目标：j1..j6 绝对角 + j7 夹爪开合；底盘字段只解析、不参与控制。
	   adopt = false 表示这一帧只解析、不下发（帧内使能字节为 0，或 S2 不在上位机档），
	   但调试镜像仍然刷新 —— 这样即使“机械臂不动”也能看出上位机到底发了什么。
	   目标即编码器连续角，与上行同一约定（上位机已把 enc_vertical 偏置加好），
	   固件不做换算；圈次对齐由 SetStreamTarget/SetTarget 内部的 UnwrapRadNear 完成。
	   j1..j6 走 SetStreamTarget：上位机是按固定频率连续刷新目标的，必须允许规划段
	   的终点速度非 0，否则每帧都要求“到新点刹停”，参考会反复加减速、抖动且滞后大。
	   j7 是夹爪：命令字节按**有符号**解释（> 0 抓紧 / < 0 张开 / 0 保持），
	   只切换两个预设位，走 Gripper_Command() → Gripper_Update() 直发驱动器，
	   不经过 Joint 的规划/PID（原因见 roboarm.hpp 顶部的夹爪说明）。 */
	bool ApplyFrame(const uint8_t* const frame, const int8_t gripper_command, const bool adopt)
	{
		/* 帧里为底盘保留的 12 字节：本机没有底盘，解析后丢弃（保持与参考工程同一帧长） */
		chassis_command_s chassis_command {};
		std::memcpy(&chassis_command, &frame[rx_chassis], sizeof(chassis_command));
		static_cast<void>(chassis_command);

		float target[host_axis_count] {};
		std::memcpy(target, &frame[rx_angles], sizeof(target));

		/* 调试镜像：无条件记下“这一帧解析出来是什么”，采纳与否由 host_debug_adopted 区分。
		   （存的是报文原值，限位/圈次处理后的每轴目标看 jN_left.target_pos）
		   命令为 0（保持）时，镜像里显示当前实际生效的预设位。 */
		host_debug_enable = frame[rx_enable];
		host_debug_gripper = static_cast<uint8_t>(gripper_command);
		host_debug_adopted = 0U;
		for (uint8_t i = 0U; i < host_axis_count; ++i)
		{
			host_debug_target[i] = target[i];
		}
		const bool gripper_close_now = (gripper_command > 0) ||
			((gripper_command == 0) && (gripper_close_target != 0U));
		host_debug_target[host_axis_count] =
			gripper_close_now ? Gripper_ClosePosition() : Gripper_OpenPosition();

		if (!adopt) { return false; }
		for (uint8_t i = 0U; i < host_axis_count; ++i)
		{
			if (!std::isfinite(target[i])) { return false; }  /* 任一轴非法就整帧丢弃 */
		}
		for (uint8_t i = 0U; i < host_axis_count; ++i)
		{
			joints[i]->SetStreamTarget(target[i]);
		}
		/* 第 7 轴（j7）是夹爪：命令有符号 —— > 0 抓紧、< 0 张开、0 保持（不改状态），
		   直接切夹爪目标位（不经过 Joint 通路）。 */
		if (gripper_command > 0) { Gripper_Command(1U); }
		else if (gripper_command < 0) { Gripper_Command(0U); }
		host_debug_adopted = 1U;   /* 放在最后：只有真的下发过才算“采纳” */
		return true;
	}
}

/* ============================ 传输完成回调（USB 中断） ============================
   由 usbd_cdc_if.c 的 CDC_TransmitCplt_HS 调用：把发完的缓冲区从 reading 收回 empty。 */
extern "C" void Host_TxComplete(void)
{
	for (uint8_t i = 0U; i < 2U; ++i)
	{
		if (HasState(i, tx_state_e::reading)) { SetState(i, tx_state_e::empty); }
	}
}

/* ============================ 协议处理入口（由 Host_Task 调用） ============================
   USB 中断只负责把字节搬进环形队列；这里做帧同步、校验、拆包、组包、发送。
   环形队列是单生产者/单消费者无锁结构，所以这里的 Get/GetBatch 窥视 + Pop/PopBatch
   提交是安全的：校验通过才真正推进读指针，中断侧可以继续写。
   非阻塞：处理完队列里当前的完整帧后立即返回（半截帧留给下一次中断唤醒）。 */
extern "C" void Host_Poll(void)
{
	uint8_t rx_frame[HOST_RX_FRAME_LEN] {};
	uint8_t head = 0U;
	taskENTER_CRITICAL();
	LatchCommandLeaseExpiry(HAL_GetTick());
	taskEXIT_CRITICAL();

	while (host_usb_queue.Get(head))
	{
		/* 字节级重同步：不是帧头就丢 1 字节继续找 */
		if (head != magic_byte)
		{
			(void)host_usb_queue.Pop(head);
			continue;
		}
		/* 定长帧还没收全：等下一次中断唤醒 */
		if (host_usb_queue.GetBatch(rx_frame, HOST_RX_FRAME_LEN) != HOST_RX_FRAME_LEN)
		{
			break;
		}
		/* 同步字 + 帧尾三重确认，失败则只丢这 1 字节 */
		if (rx_frame[rx_sync_0] != sync_byte_0 || rx_frame[rx_sync_1] != sync_byte_1
			|| rx_frame[rx_trailer] != trailer_byte)
		{
			(void)host_usb_queue.Pop(head);
			continue;
		}
		/* 校验通过：真正消费这一帧 */
		(void)host_usb_queue.PopBatch(rx_frame, HOST_RX_FRAME_LEN);
		++host_rx_frames;

		/* A zero enable byte is a stop/latch request without changing frame layout. */
		taskENTER_CRITICAL();
		const uint32_t now = HAL_GetTick();
		LatchCommandLeaseExpiry(now);
		if (rx_frame[rx_enable] == 0U)
		{
			host_safety_armed = 0U;
			host_command_seen = 0U;
			host_disabled_since_reset = 1U;
			host_safety_disable_event = 1U;
		}
		else if ((host_command_expired == 0U) &&
			(host_disabled_since_reset != 0U) &&
			(Remote_IsHostControl() != 0U) && TargetsAreFinite(rx_frame))
		{
			host_enable_after_disabled = 1U;
			host_command_tick = HAL_GetTick();
			host_command_seen = 1U;
		}
		taskEXIT_CRITICAL();

		/* 夹爪命令（有符号 ±1，0 = 保持）：每帧都记录，上行原样回显 */
		last_gripper_command = static_cast<int8_t>(rx_frame[rx_gripper]);

		/* S2 上拨才是上位机位置模式；其它档位维持遥控速度映射，忽略本帧目标
		   （仍然回状态包，上位机可据此知道当前没有控制权）。
		   两种情形都调一次 ApplyFrame：逐轴目标照样解析并写入调试镜像，
		   只是 adopt = false 时不下发。 */
		const bool adopt = (rx_frame[rx_enable] != 0U) &&
			(Remote_IsHostControl() != 0U) && (host_safety_armed != 0U);
		if (ApplyFrame(rx_frame, last_gripper_command, adopt))
		{
			host_command_tick = HAL_GetTick();
			host_command_seen = 1U;
		}
		SendStateResponse();
	}
}

/* ============================ 只读观测接口 ============================
   Host_IsOnline is diagnostic only. Motion authorization uses the separate
   100 ms accepted-command lease below and the latched MCU safety state. */
extern "C" uint8_t Host_IsOnline(void)
{
	if (host_link_tick == 0U) { return 0U; }  /* 上电后还没收到过任何 USB 数据 */
	return ((HAL_GetTick() - host_link_tick) <= HOST_LINK_TIMEOUT_MS) ? 1U : 0U;
}

extern "C" uint8_t Host_IsCommandFresh(void)
{
	uint8_t fresh;
	taskENTER_CRITICAL();
	LatchCommandLeaseExpiry(HAL_GetTick());
	fresh = (host_command_seen != 0U && host_command_expired == 0U) ? 1U : 0U;
	taskEXIT_CRITICAL();
	return fresh;
}

extern "C" void Host_ResetSafetyHandshake(void)
{
	taskENTER_CRITICAL();
	host_command_seen = 0U;
	host_command_tick = 0U;
	host_safety_armed = 0U;
	host_disabled_since_reset = 0U;
	host_enable_after_disabled = 0U;
	host_safety_disable_event = 0U;
	host_command_expired = 0U;
	taskEXIT_CRITICAL();
}

extern "C" void Host_SetSafetyArmed(uint8_t armed)
{
	taskENTER_CRITICAL();
	host_safety_armed = (armed != 0U) ? 1U : 0U;
	taskEXIT_CRITICAL();
}

extern "C" uint8_t Host_HasDisabledSinceSafetyReset(void)
{
	uint8_t seen;
	taskENTER_CRITICAL();
	seen = host_disabled_since_reset;
	taskEXIT_CRITICAL();
	return seen;
}

extern "C" uint8_t Host_HasEnableAfterDisabled(void)
{
	uint8_t seen;
	taskENTER_CRITICAL();
	seen = host_enable_after_disabled;
	taskEXIT_CRITICAL();
	return seen;
}

extern "C" uint8_t Host_ConsumeSafetyDisableEvent(void)
{
	uint8_t event;
	taskENTER_CRITICAL();
	event = host_safety_disable_event;
	host_safety_disable_event = 0U;
	taskEXIT_CRITICAL();
	return event;
}

extern "C" uint32_t Host_GetRxFrameCount(void)
{
	return host_rx_frames;
}

extern "C" uint32_t Host_GetRxDropCount(void)
{
	return host_rx_drop;
}

extern "C" uint32_t Host_GetTxDropCount(void)
{
	return host_tx_drop;
}
