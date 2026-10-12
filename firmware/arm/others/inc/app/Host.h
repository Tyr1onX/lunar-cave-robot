#pragma once

#include <stdint.h>

/* 上位机 USB-CDC 链路。三个文件各管一段，互不越界：
     · others/src/callcallback/usb_callback.cpp —— USB 中断钩子，只搬字节 + 置 HOST_RX_FLAG
     · others/src/thread/task_host.cpp          —— Host_Task 任务体，等标志 + 调 Host_Poll()
     · others/src/app/Host.cpp（本头文件）       —— 协议：帧同步/校验/拆包/执行/组包/发送
   ---------------------------------------------------------------------------
   帧格式（收发包格式与参考工程 task_host.cpp 一致，详见 Host.cpp 顶部）：
     下行 46B：A5 | 3x float32 底盘速度(vx,vy,wz，占位，解析后丢弃) | 4A B5 | 使能字节
               | 4B 预留 | 6x float32 目标关节角(j1..j6) | 夹爪命令 | 4B
     上行 35B：5A 01 4A B5 | inited | count(4B) | 6x float32 编码器连续角(j1..j6) | 夹爪命令 | 4B
   ⭐ 收发双向都是【编码器弧度】这一个约定，固件不做任何角度换算：
     上行 = Joint::PosUnwrapped()，下行目标即同一字段的目标值。上位机持有「整臂完全竖直」
     姿态下各轴的编码器值 enc_vertical[i] 作为偏置，自行换算：
       发目标 q_des(相对竖直) → target = q_des + enc_vertical[i]
       收反馈 enc            → q_meas = enc   - enc_vertical[i]
     ⚠️ 偏置表必须取自本协议的上行字段（pos_unwrapped），不能取电机手册的单圈读数
        （可能差 2πk）；换了编码器零点或改了 variables.cpp 的 offset 后需重新标定。
     下行目标固件内部会 UnwrapRadNear(requested, pos_unwrapped) 圈次对齐，
     上位机多绕/少绕 2πk 也不会被拒，不用自己管圈次。
   目标角经 Joint::SetStreamTarget()（流式入口）下发；该入口允许规划段的终点速度
   非 0（取目标自身的运动速度），因此上位机可以按固定频率连续刷新目标而不会
   被“每段都刹停”的三次曲线拖成抖动。夹爪（j7）仍用点到点的 SetTarget。 */
#define HOST_RX_FRAME_LEN    46U
#define HOST_TX_RESPONSE_LEN 35U

/* 第 7 轴（j7）按夹爪处理：下行 [44] 夹爪命令字节是**有符号**量
   —— +1 = 抓紧、-1 = 张开、0 = 保持上一状态；上行 [33] 用同一约定回显。
   ⚠️ 不要再按「0 = 张开 / 非 0 = 抓紧」判：那样 +1 与 -1 都非 0，会被判成同一个动作，
   夹爪将永远打不开。
   两个预设位与 kp/kd 统一定义在 others/src/thread/variables.cpp 的夹爪参数（不在本文件重复定义，
   避免两处数值不一致）。 */

/* USB 中断 -> Host 任务的环形缓冲容量（实际可用 HOST_RX_QUEUE_SIZE - 1 字节） */
#define HOST_RX_QUEUE_SIZE   512U

/* 链路在线判据窗口；仅用于状态观测，不参与 hold/失能策略 */
#define HOST_LINK_TIMEOUT_MS 500U

/* USB 中断收到数据后置起的任务标志：中断侧（usb_callback.cpp）与任务侧（task_host.cpp）共用一处定义 */
#define HOST_RX_FLAG 0x00000001U

/* ---- 调试镜像：最近一帧解析完成的下行目标（只写不读，不参与任何控制）----
   每解析通过一帧就刷新一次（不论这一帧最终会不会被采纳），专门给调试器看：
     host_debug_target[0..5] = j1..j6 目标角 (rad)，原样来自报文字段；
     host_debug_target[6]    = j7 夹爪目标位 (rad)，由夹爪命令字节选出的预设位
                               （达妙上位机的 pos 刻度，见 variables.cpp 的夹爪参数）；
     host_debug_enable       = 该帧的使能字节（0 = 只回状态不改目标）；
     host_debug_gripper      = 该帧的夹爪命令字节（0 = 张开，非 0 = 闭合）；
     host_debug_adopted      = 该帧最后有没有真的下发（使能非 0 && S2 上拨 && 数值合法）。
   注意：这里存的是**报文原值**，不是限位/圈次处理后的值；
   控制任务消费后的每轴目标看 jN_left.target_pos / hold_pos / pos_ref。 */
#define HOST_DEBUG_DOF 7U

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 中断侧：只在 USB 中断（CDC_Receive_HS）里调用，且必须在重新挂接接收之前调用 ---- */
void Host_UsbReceive(const uint8_t* data, uint32_t length);

/* ---- 中断侧：只在 USB 传输完成中断（CDC_TransmitCplt_HS）里调用 ----
   双缓冲状态机的一环：把已经发完的缓冲区从 reading 收回 empty。 */
void Host_TxComplete(void);

/* ---- 任务侧：由 Host_Task 在收到 HOST_RX_FLAG 后调用 ----
   把环形队列里积压的字节按协议解析：同步、校验、拆包、执行目标、回状态包。
   非阻塞，一次调用处理完当前队列里的完整帧后立即返回。 */
void Host_Poll(void);

/* 以下四个只读接口供遥测/调试使用，可被任意任务调用 */
uint8_t Host_IsOnline(void);
uint32_t Host_GetRxFrameCount(void);
uint32_t Host_GetRxDropCount(void);
uint32_t Host_GetTxDropCount(void);   /* 发送侧因缓冲区被占而丢弃的上行帧数 */

/* ---- 调试镜像（定义在 variables.cpp）----
   ⚠️ 故意**不加 volatile**：µVision 的 Watch 窗口展不开 volatile 数组，Keil 里看不到内容。
   防优化改用定义处的 __attribute__((used))，原因见 variables.cpp。 */
extern float host_debug_target[HOST_DEBUG_DOF];
extern uint8_t host_debug_enable;
extern uint8_t host_debug_gripper;
extern uint8_t host_debug_adopted;

#ifdef __cplusplus
}
#endif
