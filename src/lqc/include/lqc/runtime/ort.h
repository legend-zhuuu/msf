#ifndef LQC_RUNTIME_ORT_H_
#define LQC_RUNTIME_ORT_H_

#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace lqc {
namespace runtime {

class Runtime {
 public:
  explicit Runtime(std::string device = "cpu");
  virtual ~Runtime();
  Runtime(Runtime &) = delete;
  Runtime(const Runtime &) = delete;
  operator bool() const { return core_; }

  virtual void clearAll();
  virtual void loadModel(const std::string &path);
  virtual void run();
  std::vector<std::string> getInputNames() const;
  bool hasInput(const std::string &name) const;
  std::size_t getInputIdx(const std::string &name, bool assert = false) const;
  void setInput(const std::string &name, float *data);
  void setInput(std::size_t idx, float *data);
  std::size_t getInputSize(const std::string &name) const;
  std::size_t getInputSize(std::size_t idx) const;

  std::vector<std::string> getOutputNames() const;
  bool hasOutput(const std::string &name) const;
  std::size_t getOutputIdx(const std::string &name, bool assert = false) const;
  const float *getOutput(const std::string &name) const;
  const float *getOutput(std::size_t idx) const;
  std::size_t getOutputSize(const std::string &name) const;
  std::size_t getOutputSize(std::size_t idx) const;

 protected:
  template<typename T>
  inline Ort::Value createTensor(
      T *data, size_t size, const std::vector<int64_t> &shape) const;

  size_t num_in_ = 0, num_out_ = 0;
  std::vector<std::vector<int64_t>> in_shapes_, out_shapes_;
  std::vector<int64_t> in_sizes_, out_sizes_;
  std::vector<char *> in_names_, out_names_;
  std::vector<Ort::Value> inputs_, outputs_;
  std::vector<std::vector<float>> in_data_, out_data_;

 private:
  std::string device_;
  Ort::MemoryInfo memory_info_{nullptr};
  Ort::RunOptions run_options_{nullptr};
  Ort::Session *core_{nullptr};
};

template<typename T>
Ort::Value Runtime::createTensor(
    T *data, size_t size, const std::vector<int64_t> &shape) const {
  return Ort::Value::CreateTensor<T>(
      memory_info_, data, size, shape.data(), shape.size());
}

class MLP : public Runtime { using Runtime::Runtime; };

class RNN : public Runtime {
 public:
  using NamePairs = std::initializer_list<std::pair<std::string, std::string>>;
  RNN(NamePairs param_names = {{"h0", "hn"}, {"c0", "cn"}},
      const std::string &device = "cpu");

  void loadModel(const std::string &path) override;
  void clearAll() override;
  void clearStates();
  void run() override;

 private:
  std::vector<std::pair<std::string, std::string>> param_names_;
  std::vector<std::pair<std::size_t, std::size_t>> hidden_idx_;
  size_t num_hidden_;
};

}  // namespace runtime
}  // namespace lqc

#endif  // LQC_RUNTIME_ORT_H_
