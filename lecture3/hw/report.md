# 项目理解报告

请尽量使用自己的语言回答以下问题。可以引用少量关键代码或伪代码，但不要只粘贴实现。
完成一节后删除该节末尾的待填写标记；本地检查会拒绝仍有未完成章节的报告。

## 1. 图像生命周期与所有权

解释本项目中图像源为什么会复用缓冲区，以及 `cv::Mat` 的普通复制对底层像素数据
意味着什么。说明你的修改让一个 `Frame` 在进入队列后拥有什么，并解释为何后续读取
不会再改变它。

ImageSequenceSource 中的 buffer_ 被设计成一个可重复使用的内部缓冲区，用来模拟相机反复使用同一块图像内存的情况。每次读取新的图像后，都会通过 raw.copyTo(buffer_) 把新内容写入这块缓冲区。
cv::Mat 普通赋值并不会复制一整份像素数据。例如：
frame.image = buffer_;
只会让两个 cv::Mat 对象共享同一块底层像素数据。因此，当下一次 next() 把新图像写入 buffer_ 时，之前已经生成的 Frame 也可能看到被修改后的内容。
我将其修改为：
frame.image = buffer_.clone();
clone() 会为当前帧创建一份独立的像素数据，因此每个进入队列的 Frame 都拥有自己的图像内容，不再依赖后续会被复用的 buffer_。这样即使图像源继续读取下一帧，之前帧中的像素也不会发生变化。

## 2. 并发处理与恰好一次

结合 `BlockingQueue` 的 `push`、`pop` 和 `close` 行为，解释多个 worker 如何分工。
为什么你的实现既不会漏掉已经入队的帧，也不会重复处理同一帧？输入耗尽时，正在等待
以及仍在处理数据的 worker 分别会怎样？

producer 每成功读取一个 Frame，就调用一次：
queue_.push(frame);
把它加入BlockingQueue。多个worker都执行queue_.pop(frame)，但一个队列元素在被一个worker取出后就会从队列中移除，因此同一帧不会同时被两个worker取到。
所以虽然多个worker并发工作，但它们处理的是队列中的不同元素。producer对每个输入帧只执行一次push，而每次成功的pop又只取走一个元素，因此已经入队的帧既不会被重复处理，也不会因为多个worker而被漏掉。
当输入全部读取完成后，producer调用：
queue_.close();
关闭队列并唤醒可能正在等待的worker。此时如果队列中仍有未处理的帧，worker会继续把剩余帧取完；正在处理某一帧的worker会先完成当前处理。等队列已经关闭并且没有剩余元素后，pop()返回失败，worker的循环结束，从而正常退出。
因此关闭队列不是立即丢弃剩余数据，而是表示“不再产生新的帧”，已经进入队列的数据仍会被消费完。

## 3. 共享统计数据

指出哪些线程会读写 `Statistics`。解释原实现中的竞争为什么可能导致错误结果，并说明
你的同步方案提供了什么保证。还应说明取得快照时为什么是安全的。

Statistics 会被多个线程共同访问。producer会调用onProduced()，多个worker会调用onProcessed()、onSaved() 和onCorrupted()，main线程还可能通过snapshot()读取当前统计结果。
原来的计数更新并不是一个不可分割的操作，例如deliberatelySlowIncrement()实际经历了“读取旧值—等待—写回新值”的过程。如果两个worker 同时读取到同一个旧值，就可能分别写回相同的新值，导致一次更新丢失。这就是数据竞争。
我的实现给整个 Statistics 增加了一把：
mutable std::mutex mutex_;
每个修改计数的函数都使用：
std::lock_guard<std::mutex> lock(mutex_);
保证同一时间只有一个线程能够修改这些统计数据。lock_guard离开作用域时会自动解锁，这也符合课件中用RAII管理mutex的方式。
snapshot()也使用同一把锁。这样在读取produced_、processed_、saved_ 和corrupted_的过程中，其他线程不能同时修改这些值，因此得到的是同一时刻的一组一致统计结果，而不是几个不同时间点的数据拼在一起。
## 4. 线程关闭协议

分别描述以下两条路径中的事件顺序，并解释为什么不会发生 `std::terminate`、悬空访问
或永久等待：

1. 调用者执行 `start()` 后显式调用 `wait()`；
2. 调用者执行 `start()` 后不调用 `wait()`，直接让 `Pipeline` 析构。

如果你的实现允许某个生命周期方法被重复调用，也请说明其行为；如果不允许，请说明前置条件。

1.显式调用 wait()
调用start() 后，程序会启动多个worker线程和一个producer线程。
producer不断产生帧并放入队列。当输入耗尽后，producer调用queue_.close()，随后结束。
调用：wait();时，main线程首先检查producer是否 joinable()，如果是就执行join()，等待producer完全退出。之后依次检查每一个 worker，并对仍可join的线程调用join()。
worker在队列关闭后仍会处理已经入队的剩余帧，等到队列关闭且为空时退出循环。因此最终所有worker都能够结束，wait()才返回。
2.不显式调用wait()，直接析构
我将析构函数修改为：
Pipeline::~Pipeline()
{
    wait();
}
因此即使调用者没有主动执行 wait()，对象销毁时也会自动等待producer和所有worker结束。
这很重要，因为线程中的lambda捕获了 this，运行过程中会访问Pipeline内部的 queue_、statistics_、source_ 等成员。如果 Pipeline已经被销毁而线程还在运行，就可能访问已经失效的对象。
析构函数先调用wait()，保证所有线程已经结束，然后才继续销毁Pipeline的成员，因此不会出现线程继续访问已经销毁对象的情况，也不会留下仍然joinable的std::thread而触发std::terminate。这也对应本节课强调的线程生命周期问题。 Lecture3 Hello Modern C++
我的实现没有设计为对同一个 Pipeline 重复调用 start()，前置条件是一个 Pipeline 对象只启动一次。但是 wait() 可以重复调用，因为其中会先检查：
thread.joinable()
已经执行过 join() 的线程不再是 joinable，因此之后再次调用 wait() 不会重复 join，也不会产生错误。


