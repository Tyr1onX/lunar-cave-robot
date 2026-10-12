#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

/* 单生产者/单消费者无锁环形队列（对应参考工程的 Ring_Queue）。
   生产者：USB 中断里只调 PushBatch；
   消费者：任务里只调 Get/GetBatch/Pop/PopBatch。
   两端各自只写自己的索引（生产者写 tail、消费者写 head），用 acquire/release 配对，
   因此 ISR 侧不需要关中断。

   容量 = N - 1：留一格空位，用来区分"空"和"满"，避免额外一个计数变量。
   N 取 2 的幂时索引回绕也只是一次比较，不带除法。

   ---- 显式的读写状态机（与上行双缓冲那套同风格，方便看、方便调试）----

     写侧（生产者，只在 ISR 跑）：
       idle ──PushBatch 进入──► writing ──release 发布 tail_──► idle
       （空间不足时 writing → idle，且一个字节都不写）

     读侧（消费者，只在任务跑）：
       idle ──Get/GetBatch 窥视──► peeking ──Pop/PopBatch──► committing ──► idle
       窥视失败（队列空/不够长）→ 直接回 idle，读指针不动；
       窥视成功后会停在 peeking，直到 Pop/PopBatch 真正提交或丢弃。

     为什么窥视和提交必须分两步：校验帧头/同步字期间不能推进读指针（中断侧还在写），
     只有整帧校验通过才 PopBatch 提交；校验失败只 Pop 1 字节重新找帧头。

   注意：两个状态变量**不参与正确性判断**，正确性完全由 head_/tail_ 的 acquire/release
   配对保证；它们只是把"两端现在各在哪一步"显式记下来，可在调试器里直接观察。 */
template <typename T, uint16_t N>
class Ring_Queue
{
	static_assert(N >= 2U, "Ring_Queue: N 太小");
	static_assert(std::is_trivially_copyable<T>::value,
		"Ring_Queue: 元素类型必须可平凡拷贝");
	/* 生产者跑在中断里，索引操作必须真正无锁（Cortex-M 上 16 位对齐访问满足）。
	   若将来有人把索引改成 64 位或换个目标，这里会直接编译失败，避免在中断里跑加锁实现。 */
	static_assert(std::atomic<uint16_t>::is_always_lock_free,
		"Ring_Queue: 索引原子操作必须无锁，否则不得在 ISR 里使用");

	public:
	/* 两端的显式状态（见类头说明；只用于可读性与调试观测） */
	enum class writer_state_e : uint8_t { idle = 0U, writing = 1U };
	enum class reader_state_e : uint8_t { idle = 0U, peeking = 1U, committing = 2U };

	/* ---------------- 生产者侧（只在 ISR 调用） ---------------- */

	/* 批量写入；空间不足时整体不写并返回 false（不做半截写入）
	   状态机：idle → writing → （release 发布 tail_）→ idle */
	bool PushBatch(const T* data, uint16_t count)
	{
		if (data == nullptr || count == 0U) { return false; }

		writer_state_ = static_cast<uint8_t>(writer_state_e::writing);
		const uint16_t head = head_.load(std::memory_order_acquire);
		uint16_t tail = tail_.load(std::memory_order_relaxed);
		if (count > FreeSpace(head, tail))
		{
			writer_state_ = static_cast<uint8_t>(writer_state_e::idle);  /* 空间不足：整体不写 */
			return false;
		}

		for (uint16_t i = 0U; i < count; ++i)
		{
			buffer_[tail] = data[i];
			tail = Next(tail);
		}
		tail_.store(tail, std::memory_order_release);  /* 发布：此后这些字节对读侧可见 */
		writer_state_ = static_cast<uint8_t>(writer_state_e::idle);
		return true;
	}

	/* ---------------- 消费者侧（只在任务调用） ---------------- */

	/* 窥视 1 字节，不推进读指针（idle → peeking；队列空则直接回 idle） */
	bool Get(T& value) const
	{
		reader_state_ = static_cast<uint8_t>(reader_state_e::peeking);
		const uint16_t head = head_.load(std::memory_order_relaxed);
		if (head == tail_.load(std::memory_order_acquire))
		{
			reader_state_ = static_cast<uint8_t>(reader_state_e::idle);
			return false;
		}
		value = buffer_[head];
		return true;  /* 保持 peeking，等 Pop/PopBatch 提交 */
	}

	/* 窥视 count 个元素，不推进读指针；返回实际取出数量（不足时为 0，不做半截取出） */
	uint16_t GetBatch(T* data, uint16_t count) const
	{
		if (data == nullptr || count == 0U) { return 0U; }

		reader_state_ = static_cast<uint8_t>(reader_state_e::peeking);
		const uint16_t head = head_.load(std::memory_order_relaxed);
		const uint16_t tail = tail_.load(std::memory_order_acquire);
		if (Used(head, tail) < count)
		{
			reader_state_ = static_cast<uint8_t>(reader_state_e::idle);
			return 0U;
		}

		uint16_t idx = head;
		for (uint16_t i = 0U; i < count; ++i)
		{
			data[i] = buffer_[idx];
			idx = Next(idx);
		}
		return count;  /* 保持 peeking，等 Pop/PopBatch 提交 */
	}

	/* 消费 1 字节（peeking → committing → idle） */
	bool Pop(T& value)
	{
		if (!Get(value)) { return false; }
		reader_state_ = static_cast<uint8_t>(reader_state_e::committing);
		head_.store(Next(head_.load(std::memory_order_relaxed)), std::memory_order_release);
		reader_state_ = static_cast<uint8_t>(reader_state_e::idle);
		return true;
	}

	/* 消费 count 个元素（同时把数据拷出）；返回实际取出数量，不足时原地不动 */
	uint16_t PopBatch(T* data, uint16_t count)
	{
		if (GetBatch(data, count) != count) { return 0U; }
		reader_state_ = static_cast<uint8_t>(reader_state_e::committing);
		uint16_t head = head_.load(std::memory_order_relaxed);
		for (uint16_t i = 0U; i < count; ++i) { head = Next(head); }
		head_.store(head, std::memory_order_release);  /* 提交：这些槽位交还给生产者 */
		reader_state_ = static_cast<uint8_t>(reader_state_e::idle);
		return count;
	}

	uint16_t Size(void) const
	{
		return Used(head_.load(std::memory_order_relaxed), tail_.load(std::memory_order_acquire));
	}

	static constexpr uint16_t Capacity(void) { return static_cast<uint16_t>(N - 1U); }

	/* ---- 调试观测：两端当前状态（配合 Size() 就能看完整状态） ---- */
	writer_state_e WriterState(void) const { return static_cast<writer_state_e>(writer_state_); }
	reader_state_e ReaderState(void) const { return static_cast<reader_state_e>(reader_state_); }

	/* 仅在两端都停用时调用（例如初始化） */
	void Clear(void)
	{
		head_.store(0U, std::memory_order_relaxed);
		tail_.store(0U, std::memory_order_relaxed);
		writer_state_ = static_cast<uint8_t>(writer_state_e::idle);
		reader_state_ = static_cast<uint8_t>(reader_state_e::idle);
	}

	private:
	static uint16_t Next(const uint16_t idx) { return (idx + 1U < N) ? static_cast<uint16_t>(idx + 1U) : 0U; }

	static uint16_t Used(const uint16_t head, const uint16_t tail)
	{
		return (tail >= head) ? static_cast<uint16_t>(tail - head)
		                      : static_cast<uint16_t>(N - head + tail);
	}

	static uint16_t FreeSpace(const uint16_t head, const uint16_t tail)
	{
		return static_cast<uint16_t>(N - 1U - Used(head, tail));
	}

	std::atomic<uint16_t> head_{0U};  /* 只有消费者修改 */
	std::atomic<uint16_t> tail_{0U};  /* 只有生产者修改 */
	T buffer_[N]{};

	/* 显式读写状态机（见类头说明）：只为可读性与调试可见性，不参与正确性判断。
	   reader_state_ 用 mutable：Get/GetBatch 语义上是 const（不改数据），
	   只是顺带记录"当前正在窥视"，以便从外部观察消费者停在哪一步。 */
	mutable volatile uint8_t reader_state_ = static_cast<uint8_t>(reader_state_e::idle);
	volatile uint8_t writer_state_ = static_cast<uint8_t>(writer_state_e::idle);
};
