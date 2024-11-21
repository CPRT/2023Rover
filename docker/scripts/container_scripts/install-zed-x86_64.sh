# Based on https://github.com/stereolabs/zed-docker

# Extract ubuntu release year from /etc/lsb-release
# Expects "/etc/lsb-release" to contain a line similar to "DISTRIB_RELEASE=20.04"
export UBUNTU_RELEASE_YEAR="$(grep -o -P 'DISTRIB_RELEASE=.{0,2}' /etc/lsb-release | cut -d= -f2)"

# Extract cuda major and minor version from nvcc --version
# Expects "nvcc --version" to contain a line similar to "release 11.8"
export CUDA_MAJOR="$(nvcc --version | grep -o -P ' release .{0,4}' | cut -d. -f1 | cut -d ' ' -f3)"
export CUDA_MINOR="$(nvcc --version | grep -o -P ' release .{0,4}' | cut -d. -f2)"


# Download dependencies for zed SDK installation RUN file
sudo apt-get update -y || true
sudo apt-get install --no-install-recommends lsb-release wget less udev sudo zstd build-essential cmake libpng-dev libgomp1 -y

# TODO: Remove this when zed-ros2-wrapper has a compatible version with ZED_SDK 4.1 (which supports cuda 12.2).
# CUDA_MAJOR=12
# CUDA_MINOR=1

# Download zed SDK installation RUN file to /tmp directory
ZED_SDK_DOWNLOAD_LINK=https://download.stereolabs.com/zedsdk/4.2/cu${CUDA_MAJOR}${CUDA_MINOR%.*}/ubuntu${UBUNTU_RELEASE_YEAR}
cd /tmp
wget -q -O ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run "$ZED_SDK_DOWNLOAD_LINK"
chmod +x ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run ; ./ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run -- silent skip_od_module skip_cuda

# Symlink required for zed SDK, based on 
# https://github.com/stereolabs/zed-docker/blob/fd514606174d8bb09f21a229f1099205b284ecb6/4.X/ubuntu/devel/Dockerfile#L24
sudo ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so

# Cleanup
rm ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run
sudo rm -rf /var/lib/apt/lists/*