FROM arm64v8/ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-message-filters \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    build-essential \
    cmake \
    git \
    pkg-config \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libspdlog-dev \
    libglew-dev \
    freeglut3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    xorg-dev \
    libblas-dev \
    liblapack-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    && rm -rf /var/lib/apt/lists/*

# Set compiler flags for ARM optimization
ENV CMAKE_CXX_FLAGS="-O3 -march=native -mtune=native"
ENV CMAKE_C_FLAGS="-O3 -march=native -mtune=native"

# Set working directory
WORKDIR /workspace

# Copy source code
COPY . /workspace/src/vio_ros_wrapper/

# Initialize submodules
RUN cd /workspace/src/vio_ros_wrapper && \
    git submodule update --init --recursive

# Build the package with ARM optimizations
RUN cd /workspace && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS}" \
        -DCMAKE_C_FLAGS="${CMAKE_C_FLAGS}" \
        --parallel-workers 2

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set environment variables for OpenGL (for GUI applications)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Expose common ROS ports
EXPOSE 11311

# Default command
CMD ["/bin/bash"]