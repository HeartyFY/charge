#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

#include <atomic>
#include <memory>
#include <optional>

template<typename T>
class DoubleBuffer {
public:
  DoubleBuffer();
  ~DoubleBuffer() = default;

  void write(const T& data);
  std::optional<T> read() const;

private:
  std::unique_ptr<T> buffer1;
  std::unique_ptr<T> buffer2;
  std::atomic<T*> writeBuffer;
  std::atomic<T*> readBuffer;
  std::atomic<bool> initialized;
};

template<typename T>
DoubleBuffer<T>::DoubleBuffer() 
  : initialized(false)
{
  buffer1 = std::make_unique<T>();
  buffer2 = std::make_unique<T>();
  writeBuffer.store(buffer1.get(), std::memory_order_relaxed);
  readBuffer.store(buffer2.get(), std::memory_order_relaxed);
}

template<typename T>
void DoubleBuffer<T>::write(const T& data) {
  T* currentBuffer = writeBuffer.load(std::memory_order_relaxed);
  *currentBuffer = data;
  
  if (!initialized.load(std::memory_order_acquire)) {
    initialized.store(true, std::memory_order_release);
  }

  T* oldReadBuffer = readBuffer.exchange(currentBuffer, std::memory_order_acq_rel);
  writeBuffer.store(oldReadBuffer, std::memory_order_release);
}

template<typename T>
std::optional<T> DoubleBuffer<T>::read() const {
  if (!initialized.load(std::memory_order_acquire)) {
    return std::nullopt;  
  }

  T* currentBuffer = readBuffer.load(std::memory_order_acquire);
  return *currentBuffer;
}

#endif // DOUBLE_BUFFER_H