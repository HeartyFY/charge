#include "thread_base.h"
#include <iostream>

ThreadBase::ThreadBase() { }

ThreadBase::~ThreadBase() {
  stop_thread();
}

void ThreadBase::start_thread() {
  if (p_thread_ == nullptr) {
    thread_pause_flag_ = false;
    thread_stop_flag_ = false;
    p_thread_ = std::make_shared<std::thread>(&ThreadBase::Run, this);

    if (p_thread_ != nullptr) {
      std::cout << "start thread!" << std::endl;
      thread_state_ = ThreadState::Running;
    }
  }
}

void ThreadBase::pause_thread() {
  if (p_thread_ != nullptr) {
    if (thread_state_ == ThreadState::Running){
      thread_pause_flag_ = true;
      thread_state_ = ThreadState::Pause;
      std::cout << "pause thread!" << std::endl;
    }
  }
}

void ThreadBase::resume_thread() {
  if (p_thread_ != nullptr) {
    if (thread_state_ == ThreadState::Pause) {
      thread_pause_flag_ = false;
      cv_.notify_all();
      thread_state_ = ThreadState::Running;
      std::cout << "resume thread!" << std::endl;
    }
  }
}

void ThreadBase::stop_thread() {
  thread_state_ = ThreadState::Stop;
  if (p_thread_ != nullptr) {
    thread_stop_flag_  = true;
    thread_pause_flag_ = false;

    cv_.notify_all();
    if (p_thread_->joinable()) {
      p_thread_->join();
    } else {
      std::cout << "unable close thread" << std::endl;
    }

    p_thread_.reset();

    if (p_thread_ == nullptr) {
      std::cout << "stop thread success!" << std::endl;
    } else {
      std::cout << "stop thread failed!" << std::endl;
    }
  }
}

void ThreadBase::Run() {
  while (!thread_stop_flag_) {
    Process();
    if (thread_pause_flag_) {
      std::unique_lock<std::mutex> thread_locker(mutex_);
      while (thread_pause_flag_) {
        cv_.wait(thread_locker);
      }
      thread_locker.unlock();
    }
  }
  thread_pause_flag_ = false;
  thread_stop_flag_  = false;
}

ThreadState ThreadBase::get_thread_state() const { return thread_state_; }
