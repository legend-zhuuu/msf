---
title: 使用Docker进行开发
---

## Prerequisites

配置完成基础的操作系统与软件系统

### OS Installation

经过测试的操作系统发行版如下：

- 开发部署：
  - Ubuntu 20.04.5 LTS
  - 银河麒麟桌面操作系统 V10 SP1

### Docker Installation

- 针对 Ubuntu 操作系统，可以直接参照 Docker [官方文档](https://docs.docker.com/engine/install/ubuntu/) `Install using the repository` 进行安装，安装完成后，建议参照 [此文档](https://docs.docker.com/engine/install/linux-postinstall/) 进行后续配置，并添加Docker [国内镜像源](https://huaweicloud.csdn.net/63312161d3efff3090b5321c.html?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2~default~BlogCommendFromBaidu~activity-1-123962707-blog-124156331.pc_relevant_3mothn_strategy_and_data_recovery&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2~default~BlogCommendFromBaidu~activity-1-123962707-blog-124156331.pc_relevant_3mothn_strategy_and_data_recovery&utm_relevant_index=1) 。
- 针对不能联网或者银河麒麟等`docker`未支持的操作系统，需要使用[安装包](https://docs.docker.com/engine/install/ubuntu/#install-from-a-package)或者[二进制包](https://docs.docker.com/engine/install/binaries/)进行离线安装。

## Downlod Sources

将工程代码仓库克隆至本地

```bash
git clone https://gitee.com/csc105_slam_group/multi_sensor_mapping.git
```

下载工程依赖

```bash
cd multi_sensor_mapping/docker/script/
./download_source.sh 
```

注意：

- 因 `docker` 代理配置问题 `git` 下载问题，选择使用`COPY` 将依赖源码复制到 `/tmp` 路径编译安装后删除。

## Build/Import Docker Container

使用如下命令构建工程的 Docker 容器(支持`amd64`和`arm64v8`架构)：

```bash
cd multi_sensor_mapping/docker/script/
./script/build.sh
```

```shell
# 在 build.sh内进行架构选择

build_image "linux/amd64" # amd64 
# build_image "linux/arm64" # arm64v8
```

构建完成后，系统中将出现名为 `slam/mapping:ubuntu18.04-amd64` 的 Docker 镜像，可以使用 `docker images` 命令进行查看：

```bash
$ docker images
REPOSITORY         TAG                       IMAGE ID       CREATED         SIZE
slam/mapping       ubuntu18.04-amd64         59c1c21d3b94   8 minutes ago   2.85GB
```

## Run/Enter Docker Container

使用如下命令创建并进入 Docker 容器：

```bash
cd multi_sensor_mapping/docker/
./script/run.sh
```

```shell
# 在 run.sh内进行架构选择

IMAGE_TO_RUN=${IMAGE_NAME:-slam/mapping:ubuntu18.04-amd64} # amd64
# IMAGE_TO_RUN=${IMAGE_NAME:-slam/mapping:ubuntu18.04-arm64v8} # amd64
```

使用该脚本运行的容器，具备以下特性：

- `--net=host`:容器使用`host`模式，容器的可访问网络与主机相同，无需网络地址转换过程，便于容器访问宿主机连接的网络端口外设；
- `--volume="/dev:/dev"`:将主机的设备信息转发至容器内，便于容器访问宿主机连接的外设；
- `--volume="${PWD}/../../../:/workspace/msm_ws/"`：映射宿主机中的当前工程目录至 Docker 容器中的`/workspace/msm_ws/src`路径；
- `--env="DISPLAY=$DISPLAY"`：将容器中的 X11 界面转发至宿主机，便于直接在宿主机中显示容器中的具有 GUI 的软件（如`rqt`、`rviz`等）；

使用 docker ps 命令，可以查看当前正在运行的 Docker 容器：

```bash
#查看当前正在运行的 Docker 容器
$ docker ps
CONTAINER ID   IMAGE                            COMMAND                  CREATED         STATUS          PORTS     NAMES
870e835d3e41   slam/mapping:ubuntu18.04-amd64   "/bin/sh -c 'service…"   3 seconds ago   Up 2 seconds              mapping
```

创建 Docker 容器后，需要再次进入时，可以使用如下命令进入已创建的 Docker 容器：

```bash
cd multi_sensor_mapping/docker/
./script/exec.sh
```

## Build inside Docker Container

进入 Docker 容器后：

- 编译项目

```bash
cd workspace/msm_ws/
catkin_make -j4
```
