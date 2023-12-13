# Multi Sensor Mapping

 欢迎来到multi-sensor-mapping的代码仓库。

- [Multi Sensor Mapping](#multi-sensor-mapping)
  - [Introduction](#introduction)
  - [Dependence](#dependence)
  - [Usage](#usage)
  - [Known Issues](#known-issues)
  - [TODO](#todo)
  - [Docker](#docker)
  - [CMake Install](#cmake-install)

## Introduction

基于多源传感器融合的建图，包括：

- 激光-IMU融合建图
- 地图后处理工具包

![建图效果](doc/pic/huaxin.png)

## Dependence

- **ubuntu 18.04**

- **ros melodic**

- **ROS依赖**

  ```bash
   # RsLidar 相关
   sudo apt-get install ros-melodic-rslidar-msgs ros-melodic-cv-bridge \
        ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-jsk-recognition-msgs \
        ros-melodic-octomap ros-melodic-octomap-msgs 
  ```

- [GTSAM](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library)
  
  ```bash
  git clone https://github.com/borglab/gtsam.git
  cd gtsam
  git checkout 4.0.2
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
  sudo make install -j8
  ```
  
- **glog**

  ```bash
  sudo apt-get install libgoogle-glog-dev
  ```

- **Ceres**

  ```bash
  # install ceres dependence
  sudo apt-get install libgoogle-glog-dev
  sudo apt-get install libatlas-base-dev
  sudo apt-get install libeigen3-dev
  sudo apt-get install libsuitesparse-dev
  sudo apt-get update

  # download ceres code
  git clone https://ceres-solver.googlesource.com/ceres-solver
  cd ceres-solver
  git checkout 2.0.0
  
  # compile
  cd ceres-solver
  mkdir build
  cd build
  cmake ..
  make -j4
  sudo make install
  ```

- **Sophus**

  ```bash
  git clone https://github.com/strasdat/Sophus.git
  cd Sophus/
  git checkout 593db47500
  mkdir build
  cd build
  cmake .. 
  make -j4
  sudo make install
  ```

- **rs_driver** (RsLiDAR packet 数据解析)：
  第一次连接`RoboSense-LiDAR`，推荐使用`wireshark`查看对应网络，`docker`镜像内也安装了该工具。

  ```bash
  sudo apt-get install libboost-dev libpcap-dev libpcl-dev libeigen3-dev
  
  git clone https://github.com/RoboSense-LiDAR/rs_driver.git
  git checkout v1.5.8
  mkdir build && cd build
  cmake .. && make -j4
  sudo make install
  ```

- **rslidar_sdk** (RoboSense LiDAR SDK for ros)

  ``` bash
  git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
  cd rslidar_sdk
  git checkout v1.5.8
  git submodule init
  git submodule update
  ```
  
- **geographiclib** (GPS相关坐标系转换)

  ```bash
  git clone https://github.com/sotex/geographiclib.git
  cd geographiclib/
  mkdir build && cd build
  cmake .. && make -j4
  sudo make install
  ```

- **zipper**

  ```bash
  git clone https://github.com/lecrapouille/zipper.git --recursive
  cd zipper
  make download-external-libs
  make compile-external-libs
  make -j`nproc --all`
  sudo make install
  ```

- **hunter_msg** (optition 测试时使用)

  ```bash
  git clone https://github.com/CosCJ/hunter_ros.git
  ```

## Usage

- **激光-IMU融合建图**

  ```bash
  roslaunch multi_sensor_mapping LidarImuMapping.launch
  ```

- **激光-轮速计融合建图**

  ```bash
  roslaunch multi_sensor_mapping LidarOdomMapping.launch
  ```

- **纯激光建图**

  ```bash
  roslaunch multi_sensor_mapping PureLidarMapping.launch
  ```

- **地图后处理工具包**

  ```bash
  roslaunch multi_sensor_mapping GridMapGeneration.launch
  ```

## Known Issues

1. 编译问题 ceres_local_param.hpp报错

   原因：Sophus库冲突

   解决方法

   ```bash
    sudo apt-get remove ros-melodic-sophus
   ```

2. 编译问题 LZ4_streamDecode_t

   ```bash
   sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
   sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak
   
   sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
   sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
   ```

3. 在cmakelists设置`-march=native`，会导致PCL点云析构的时候 core dump。第三方依赖库不能有`-march=native`的 编译选项。

4. [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)存在packet时间戳的bug,需要修改源码。
   将`lidar_driver_impl.hpp`的地182行后加上

   ```c++
    // rewrite pkt timestamp or not ?
    decoder_ptr_->enableWritePktTs((cb_put_pkt_ == nullptr) ? false : true);
    decoder_ptr_->enableWritePktTs(true)
   ```

    这是因为在rslidar_sdk中调用时，先`init()`，再注册了packet的回调函数导致的。

## TODO

- [ ] IMU预积分效果不佳，测试IMU预积分结果
- [x] 添加Log数据，把重要的分析数据保存下来
- [x] 可视化加上闭环的部分
- [ ] 基于面元特征的配准和处理
- [ ] 基于面元的GlobalBA
- [x] 支持激光packet数据
- [x] 自适应地图降采样
- [ ] 地图动态物体去除
- [x] 融合GNSS数据

## Docker

请查看[Docker文档](docs/Docker.md)

## CMake Install

请查看[CMake文档](docs/CMake.md)
