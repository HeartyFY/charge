#ifndef THREAD_BASE_H
#define THREAD_BASE_H

#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>

enum class ThreadState {
    Stop    = 0,
    Running = 1,
    Pause   = 2,
};

class ThreadBase {
public:
  ThreadBase();
  virtual ~ThreadBase();

public:
  void start_thread();
  void pause_thread();
  void resume_thread();
  void stop_thread();

  ThreadState get_thread_state() const;

private:
  void Run();

protected:
  virtual void Process() = 0;

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> thread_stop_flag_;
  std::atomic<bool> thread_pause_flag_;
  std::shared_ptr<std::thread> p_thread_ = nullptr;
  ThreadState thread_state_ = ThreadState::Stop;
};

#endif  // !THREAD_BASE_H
