# ndt-omp
欢迎来到ndt-omp的代码仓库。

## Introduction

ndt-omp是多线程加速的基于NDT的点云配准库。



## Dependence

- **PCL**



## Usage

### ROS编译

（1）打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**CATKIN**.

```cmake
#=======================================
# Compile setup (ORIGINAL, CATKIN)
#=======================================
set(COMPILE_METHOD CATKIN)
```

（2) 在ROS工作空间中**catkin_make** 编译.



### CMAKE编译

（1）打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**ORIGINAL**.

```cmake
#=======================================
# Compile setup (ORIGINAL, CATKIN)
#=======================================
set(COMPILE_METHOD ORIGINAL)
```

（2）使用cmake编译

```bash
cd ndt-omp
mkdir build
cd build
cmake ..
make -j
```



### 作为外部依赖库

