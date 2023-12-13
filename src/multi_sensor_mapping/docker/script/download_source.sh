CERES_VERSION=${1:-"2.0.0"}
SOPHUS_VERSION=${2:-"593db47500"}
GTSAM_VERSION=${3:-"4.0.3"}
RS_DRIVER_VERSION=${4:-"v1.5.8"}

mkdir ../source
cd ../source 

# downlaod ceres-solver
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout ${CERES_VERSION}
cd ..

# download geographiclib
git clone https://github.com/sotex/geographiclib.git

# download gtsam
git clone https://github.com/borglab/gtsam.git 
cd gtsam 
git checkout ${GTSAM_VERSION}
cd ..   

# download rs_driver
git clone https://github.com/RoboSense-LiDAR/rs_driver.git 
cd rs_driver 
git checkout ${RS_DRIVER_VERSION}
cd ..

# download sophus
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout ${SOPHUS_VERSION}
cd ..

# download zipper
git clone https://github.com/lecrapouille/zipper.git --recursive
cd zipper
make download-external-libs
cd ..

# download cmake
wget https://cmake.org/files/v3.21/cmake-3.21.7-linux-x86_64.tar.gz && \
tar -xvzf cmake-3.21.7-linux-x86_64.tar.gz

wget https://cmake.org/files/v3.21/cmake-3.21.7-linux-aarch64.tar.gz && \
tar -xvzf cmake-3.21.7-linux-aarch64.tar.gz