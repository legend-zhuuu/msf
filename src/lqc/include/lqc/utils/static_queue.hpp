#ifndef UTILS_STATIC_QUEUE_HPP_
#define UTILS_STATIC_QUEUE_HPP_

#include <exception>

template<typename T>
class StaticQueue {
 public:
  StaticQueue() = default;

  class iterator : public std::iterator<std::forward_iterator_tag, T> {
   public:
    explicit iterator(StaticQueue<T> *q, std::size_t idx = 0) : q_(q), idx_(idx) {}
    iterator &operator++() {
      idx_++;
      return *this;
    }
    bool operator==(iterator other) const { return idx_ == other.idx_ && q_ == other.q_; }
    bool operator!=(iterator other) const { return idx_ != other.idx_ || q_ != other.q_; }
    T &operator*() const { return q_->operator[](idx_); }
   private:
    StaticQueue<T> *q_;
    std::size_t idx_;
  };

  class const_iterator : public std::iterator<std::forward_iterator_tag, T, std::ptrdiff_t, const T *, const T &> {
   public:
    explicit const_iterator(const StaticQueue<T> *q, std::size_t idx = 0) : q_(q), idx_(idx) {}
    const_iterator &operator++() {
      idx_++;
      return *this;
    }
    bool operator==(const_iterator other) const { return idx_ == other.idx_ && q_ == other.q_; }
    bool operator!=(const_iterator other) const { return idx_ != other.idx_ || q_ != other.q_; }
    const T &operator*() const { return q_->operator[](idx_); }
   private:
    const StaticQueue<T> *q_;
    std::size_t idx_;
  };

  explicit StaticQueue(std::size_t capacity)
      : capacity_(capacity) { data_ = new T[capacity]; }

  ~StaticQueue() { delete[] data_; }
  struct IndexOutOfRange : public std::exception {
    const char *what() const noexcept override { return "Queue Index Out Of Range!"; }
  };

  struct EmptyQueue : public std::exception {
    const char *what() const noexcept override { return "Empty Queue!"; }
  };

  T &front() {
    assert_not_empty();
    return data_[front_];
  }
  const T &front() const {
    assert_not_empty();
    return data_[front_];
  }

  T &back() {
    assert_not_empty();
    return data_[(front_ + size_ - 1) % capacity_];
  }
  const T &back() const {
    assert_not_empty();
    return data_[(front_ + size_ - 1) % capacity_];
  }

  void reserve(std::size_t size) {
    if (data_ == nullptr) {
      data_ = new T[size];
      capacity_ = size;
    } else {
      T *temp = new T[size];
      std::size_t start = 0, copy_num = std::min(size, size_);
      if (size_ > size) start = size_ - size;
      for (std::size_t i = 0; i < copy_num; ++i) {
        temp[i] = operator[](start + i);
      }
      if (size_ > size) size_ = size;
      front_ = 0;
      capacity_ = size;
      delete[] data_;
      data_ = temp;
    }
  }

  void push_back(const T &item) noexcept {
    if (is_full()) front_ = (front_ + 1) % capacity_;
    else ++size_;
    data_[(front_ + size_ - 1) % capacity_] = item;
  }
  T &roll() noexcept {
    if (is_full()) {
      front_ = (front_ + 1) % capacity_;
    } else {
      ++size_;
    }
    return back();
  }

  void clear() noexcept { front_ = size_ = 0; }
  void clearall() noexcept {
    clear();
    delete[] data_;
    data_ = nullptr;
  }
  std::size_t size() const noexcept { return size_; }
  bool is_empty() const noexcept { return size_ == 0; }
  bool is_full() const noexcept { return size_ == capacity_; }

  const T &get_padded(int idx) const {
    if (idx < 0) idx = int(size_) + idx;
    if (idx < 0) return front();
    else if (idx >= size_) return back();
    else return data_[(front_ + idx) % capacity_];
  }

  const T &get(int idx, const T &default_value) const {
    if (idx < 0) idx = int(size_) + idx;
    if (idx < 0 or idx >= size_) return default_value;
    return data_[(front_ + idx) % capacity_];
  }

  T &operator[](int idx) {
    if (idx < 0) idx = int(size_) + idx;
    if (idx < 0 or idx >= size_) throw IndexOutOfRange();
    return data_[(front_ + idx) % capacity_];
  }

  T &operator[](std::size_t idx) {
    if (idx >= size_) throw IndexOutOfRange();
    return data_[(front_ + idx) % capacity_];
  }

  const T &operator[](int idx) const {
    if (idx < 0) idx = int(size_) + idx;
    if (idx < 0 or idx >= size_) throw IndexOutOfRange();
    return data_[(front_ + idx) % capacity_];
  }

  const T &operator[](std::size_t idx) const {
    if (idx >= size_) throw IndexOutOfRange();
    return data_[(front_ + idx) % capacity_];
  }

  iterator begin() { return iterator(this); }
  const_iterator begin() const { return const_iterator(this); }
  iterator end() { return iterator(this, size_); }
  const_iterator end() const { return const_iterator(this, size_); }

  friend std::ostream &operator<<(std::ostream &os, const StaticQueue<T> &queue) {
    os << "<";
    if (!queue.is_empty()) {
      os << queue.front();
      for (std::size_t i = 1; i < queue.size_; ++i) {
        os << "," << queue[i];
      }
    }
    return os << ">";
  }

 private:
  void assert_not_empty() const {
    if (is_empty()) throw EmptyQueue();
  }
  T *data_{nullptr};
  std::size_t front_ = 0, size_ = 0;
  std::size_t capacity_ = 0;
};

#endif  // UTILS_STATIC_QUEUE_HPP_
