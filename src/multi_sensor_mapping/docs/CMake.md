# CMake

生成工程部署或SDK所需的可执行文件和动态链接库

## Install

主要难点为工程依赖的第三方库，推荐新建`external`文件夹（子目录）放入第三方库源码，并在主工程通过`CMake`指令`add_subdirectory`添加第三方库路径

### 依赖

- `CMake`>=3.13

`CMake`3.13 才开始支持使用`add_subdirectory`添加子目录，故需要升级系统`Cmake`版本。
方法为下载高版本可执行文件，并通过软连接进行覆盖。

``` bash
# 查看版本
cmake --version

# 版本更新
wget https://cmake.org/files/v3.21/cmake-3.21.7-linux-x86_64.tar.gz
tar -xvzf cmake-3.21.7-linux-x86_64.tar.gz
mv cmake-3.21.7-linux-x86_64 /opt/cmake-3.21
ln -sf /opt/cmake-3.21/bin/* /usr/bin

# 验证
cmake --version
```

### 实战

现以[multi_sensor_mapping](https://gitee.com/csc105_slam_group/multi_sensor_mapping)工程举例，该工程依赖第三方库[ndt_omp](https://github.com/APRIL-ZJU/ndt_omp)

1. 在主工程目录`external`文件夹下添加第三方库源码
2. 在主工程的`CMakeLists.txt`中使用`add_subdirectory`添加子目录路径

  ``` CMake
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/ndt_omp)
  ```

3. 即可安装、链接第三方库的动态库

  ``` CMake
  install(TARGETS
    ndt_omp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ```

## 其他

`ROS`工程使用`Catkin`并存在`Install`库的[范式](http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html)，可学习并较容易地转换为`CMake`范式。

存在如下路径转换（工程所属可执行文件放置于工程名文件夹下，动态链接库放置于其外）：

- `${CATKIN_PACKAGE_LIB_DESTINATION}` -> `lib`
- `${CATKIN_PACKAGE_BIN_DESTINATION}` -> `lib/${PROJECT_NAME}`
- `${CATKIN_GLOBAL_BIN_DESTINATION}` -> `bin`
- `${CATKIN_PACKAGE_SHARE_DESTINATION}` -> `share/${PROJECT_NAME}`

### `ROS`工程范式

以[multi_sensor_localization](https://gitee.com/csc105_slam_group/multi_sensor_localization)工程举例。

``` CMake
#############
## Install ##
#############

### 可执行文件
install(TARGETS
    msl_manager_node
    matrix2extrinsic
    initial_pose_tool
    initial_pose_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### 运行所需动态链接库
### Mark libraries for installation
install(TARGETS
    msl_core msl_utils msl_frontend
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


### Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

### Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
 launch
 config
 rviz
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

![lib_tree](pic/lib_tree.png)
![share_tree](pic/share_tree.png)
