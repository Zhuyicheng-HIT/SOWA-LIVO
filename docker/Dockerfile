FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libomp-dev \
    libopencv-dev \
    libpcl-dev \
    ros-noetic-cv-bridge \
    ros-noetic-eigen-conversions \
    ros-noetic-image-transport \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["bash"]
