FROM osrf/ros:jazzy-desktop

# ========== 1. 源配置（最稳定，放最前） ==========
RUN sed -i 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g; s|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list.d/ubuntu.sources

RUN sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' /usr/share/ros-apt-source/ros2.sources && \
    sed -i '/^Types:/ s/deb-src//' /usr/share/ros-apt-source/ros2.sources

# ========== 2. 基础工具（几乎不变） ==========
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip vim nano gdb tree htop wget curl iputils-ping net-tools unzip \
    ninja-build bear pkg-config g++ \
    lsb-release gnupg software-properties-common

# ========== 3. 系统开发库（较少变） ==========
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev libeigen3-dev libevdev-dev libudev-dev libfmt-dev \
    libspdlog-dev libserial-dev libncurses-dev libxcb-cursor0

# ========== 4. X11 显示相关（取代 Xpra） ==========
# 移除 xvfb，添加 xauth 和 x11-xserver-utils
RUN apt-get update && apt-get install -y --no-install-recommends \
    xauth x11-xserver-utils x11-apps

# ========== 5. ROS 2 包（以后追加最频繁，单独成层） ==========
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-moveit \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-joint-state-publisher-gui

# ========== 6. Python 包（独立成层） ==========
RUN pip3 install --no-cache-dir --break-system-packages \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    --trusted-host pypi.tuna.tsinghua.edu.cn \
    ruff

# ========== 7. LLVM ==========
ARG CLANG_VERSION=23
ARG HTTP_PROXY
ARG HTTPS_PROXY
RUN export http_proxy="$HTTP_PROXY" https_proxy="$HTTPS_PROXY" && \
    wget https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh ${CLANG_VERSION} && \
    rm -f llvm.sh && \
    apt-get update && apt-get install -y --no-install-recommends lld-${CLANG_VERSION}

# ========== 8. Oh My Posh ==========
RUN curl -fsSL https://github.com/JanDeDobbeleer/oh-my-posh/releases/latest/download/posh-linux-amd64 \
    -o /usr/local/bin/oh-my-posh && \
    chmod +x /usr/local/bin/oh-my-posh && \
    oh-my-posh version

# ========== 9. bashrc 配置 ==========
RUN echo 'eval "$(oh-my-posh init bash --config /root/oh_my_posh.json)"' >> /root/.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash" > /etc/profile.d/ros.sh && \
    chmod +x /etc/profile.d/ros.sh

# ========== 10. 最后清理 apt 索引（缩小体积） ==========
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /workspace