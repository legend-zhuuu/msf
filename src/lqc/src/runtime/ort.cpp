#include <algorithm>
#include <iostream>
#include <numeric>

#include <lqc/runtime/ort.h>
#include <lqc/utils/utils.h>

namespace lqc {
namespace runtime {

Runtime::Runtime(std::string device) : device_(std::move(device)) {
  memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
}

Runtime::~Runtime() { Runtime::clearAll(); }

void Runtime::clearAll() {
  delete core_;
  in_shapes_.clear();
  out_shapes_.clear();
  in_sizes_.clear();
  out_sizes_.clear();
  for (auto ptr : in_names_) delete[] ptr;
  for (auto ptr : out_names_) delete[] ptr;
  in_names_.clear();
  out_names_.clear();
  inputs_.clear();
  outputs_.clear();
  in_data_.clear();
  out_data_.clear();
}

void Runtime::loadModel(const std::string &path) {
  clearAll();
  if (device_ == "cpu") {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, path.c_str());
    Ort::SessionOptions opts;
    opts.SetInterOpNumThreads(1);
    opts.SetIntraOpNumThreads(1);
    core_ = new Ort::Session(env, path.c_str(), opts);
  } else {
    // TODO: IMPLEMENT GPU INFERENCE
    throw std::runtime_error("Gpu Inference Not Implemented!");
  }

  num_in_ = core_->GetInputCount();
  num_out_ = core_->GetOutputCount();

  Ort::AllocatorWithDefaultOptions allocator;
  for (int i = 0; i < num_in_; ++i) {
    auto type_info = core_->GetInputTypeInfo(i);
    auto info = type_info.GetTensorTypeAndShapeInfo();
    auto shape = info.GetShape();
    for (auto &dim : shape) if (dim == -1) dim = 1;
    int64_t size = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<>());
    auto name = core_->GetInputNameAllocated(i, allocator);

    in_shapes_.emplace_back(std::move(shape));
    in_sizes_.push_back(size);
    std::vector<float> data(size, 0.f);
    in_data_.push_back(std::move(data));
    auto *p_name = new char[std::strlen(name.get()) + 1];
    std::strcpy(p_name, name.get());
    in_names_.push_back(p_name);
    inputs_.push_back(createTensor(in_data_[i].data(), size, in_shapes_[i]));
  }
  for (int i = 0; i < num_out_; ++i) {
    auto type_info = core_->GetOutputTypeInfo(i);
    auto info = type_info.GetTensorTypeAndShapeInfo();
    auto shape = info.GetShape();
    for (auto &dim : shape) if (dim == -1) dim = 1;
    int64_t size = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<>());
    auto name = core_->GetOutputNameAllocated(i, allocator);

    out_shapes_.emplace_back(std::move(shape));
    out_sizes_.push_back(size);
    std::vector<float> data(size, 0.f);
    out_data_.push_back(std::move(data));
    auto *p_name = new char[std::strlen(name.get()) + 1];
    std::strcpy(p_name, name.get());
    out_names_.push_back(p_name);
    outputs_.push_back(createTensor(out_data_[i].data(), size, out_shapes_[i]));
  }

  log("Model ", path);
  if (lg::inMode(lg::DBUG)) {
    std::stringstream ss;
    for (int i = 0; i < num_in_; ++i) {
      if (i != 0) ss << "\n";
      ss << "IN  " << i << " " << in_names_[i];
      const auto &shape = in_shapes_[i];
      ss << ": [" << shape[0];
      for (int j = 1; j < shape.size(); ++j) {
        ss << " x " << shape[j];
      }
      ss << "] = " << in_sizes_[i];
    }
    for (int i = 0; i < num_out_; ++i) {
      ss << "\nOUT " << i << " " << out_names_[i];
      const auto &shape = out_shapes_[i];
      ss << ": [" << shape[0];
      for (int j = 1; j < shape.size(); ++j) {
        ss << " x " << shape[j];
      }
      ss << "] = " << out_sizes_[i];
    }
    log(ss.str());
  }
}

void Runtime::run() {
  core_->Run(run_options_, in_names_.data(), inputs_.data(), in_names_.size(),
             out_names_.data(), outputs_.data(), outputs_.size());
}

std::vector<std::string> Runtime::getInputNames() const {
  std::vector<std::string> names;
  for (auto *name : in_names_) names.emplace_back(name);
  return names;
}

bool Runtime::hasInput(const std::string &name) const {
  return getInputIdx(name) != -1;
}

std::size_t Runtime::getInputIdx(const std::string &name, bool assert) const {
  auto it = std::find(in_names_.begin(), in_names_.end(), name);
  if (it == in_names_.end()) {
    if (!assert) return -1;
    LQC_ERROR("No input named `" + name + "`!")
  }
  return it - in_names_.begin();
}

void Runtime::setInput(const std::string &name, float *data) {
  setInput(getInputIdx(name, true), data);
}

void Runtime::setInput(std::size_t idx, float *data) {
  std::copy(data, data + in_sizes_[idx], in_data_[idx].data());
}

std::size_t Runtime::getInputSize(const std::string &name) const {
  return getInputSize(getInputIdx(name, true));
}

std::size_t Runtime::getInputSize(std::size_t idx) const {
  return in_sizes_[idx];
}

std::vector<std::string> Runtime::getOutputNames() const {
  std::vector<std::string> names;
  for (auto *name : out_names_) names.emplace_back(name);
  return names;
}

bool Runtime::hasOutput(const std::string &name) const {
  return getOutputIdx(name) != -1;
}

std::size_t Runtime::getOutputIdx(const std::string &name, bool assert) const {
  auto it = std::find(out_names_.begin(), out_names_.end(), name);
  if (it == out_names_.end()) {
    if (!assert) return -1;
    LQC_ERROR("No output named `" + name + "`!")
  }
  return it - out_names_.begin();
}

const float *Runtime::getOutput(const std::string &name) const {
  return getOutput(getOutputIdx(name, true));
}

const float *Runtime::getOutput(std::size_t idx) const {
  return out_data_[idx].data();
}

std::size_t Runtime::getOutputSize(const std::string &name) const {
  return getOutputSize(getOutputIdx(name, true));
}

std::size_t Runtime::getOutputSize(std::size_t idx) const {
  return out_sizes_[idx];
}

RNN::RNN(NamePairs param_names, const std::string &device)
    : Runtime(device), param_names_(param_names) {}

void RNN::loadModel(const std::string &path) {
  Runtime::loadModel(path);

  num_hidden_ = param_names_.size();
  for (const auto &name : param_names_) {
    hidden_idx_.emplace_back(
        getInputIdx(name.first, true),
        getOutputIdx(name.second, true)
    );
  }

  if (lg::inMode(lg::DBUG)) {
    log("Hidden states:");
    for (int i = 0; i < num_hidden_; ++i) {
      log(param_names_[i].first, " (IN ", hidden_idx_[i].first, ") -> ",
          param_names_[i].second, " (OUT ", hidden_idx_[i].second, ")");
    }
  }
}

void RNN::clearStates() {
  for (auto idx : hidden_idx_) {
    std::fill(in_data_[idx.first].begin(), in_data_[idx.first].end(), 0.f);
  }
}

void RNN::run() {
  Runtime::run();
  for (auto idx : hidden_idx_) {
    std::swap(inputs_[idx.first], outputs_[idx.second]);
    std::swap(in_data_[idx.first], out_data_[idx.second]);
  }
}

void RNN::clearAll() {
  Runtime::clearAll();
  num_hidden_ = 0;
  hidden_idx_.clear();
}

}  // namespace runtime
}  // namespace lqc
