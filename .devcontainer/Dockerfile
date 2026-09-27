FROM osrf/ros:lyrical-desktop

# ========== 清华 Ubuntu 源：先删旧源文件，再写新源 ==========
RUN rm -f /etc/apt/sources.list /etc/apt/sources.list.d/ubuntu.sources && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ resolute main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ resolute-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ resolute-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ resolute-security main restricted universe multiverse" >> /etc/apt/sources.list

# ========== 清华 ROS2 源：sed 替换 URL + 关掉 deb-src ==========
RUN sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
    /usr/share/ros-apt-source/ros2.sources && \
    sed -i '/^Types:/ s/deb-src//' /usr/share/ros-apt-source/ros2.sources

# ── 系统工具 + 系统库 + ROS2 包 + ruff ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ── 基础工具 ──
    python3-pip \
    vim \
    nano \
    gdb \
    tree \
    htop \
    wget \
    curl \
    iputils-ping \
    net-tools \
    unzip \
    # ── 构建工具 ──
    ninja-build \
    bear \
    pkg-config \
    g++ \
    libstdc++-16-dev \
    # ── 系统开发库 ──
    libusb-1.0-0-dev \
    libeigen3-dev \
    libevdev-dev \
    libudev-dev \
    libfmt-dev \
    libspdlog-dev \
    libserial-dev \
    libncurses-dev \
    libxcb-cursor0 \
    # ── ROS 2 包 ──
    ros-lyrical-ros2-control \
    ros-lyrical-ros2-controllers \
    ros-lyrical-xacro \
    ros-lyrical-moveit \
    ros-lyrical-moveit-setup-assistant \
    ros-lyrical-rmw-cyclonedds-cpp \
    # ── 系统管理/包管理工具 ──
    lsb-release \
    gnupg \
    software-properties-common \
    && pip3 install --no-cache-dir --break-system-packages \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    --trusted-host pypi.tuna.tsinghua.edu.cn \
    ruff \
    && rm -rf /var/lib/apt/lists/*

# ── 安装 LLVM 工具链 ──
ARG CLANG_VERSION=23
ARG HTTP_PROXY
ARG HTTPS_PROXY
RUN export http_proxy="$HTTP_PROXY" https_proxy="$HTTPS_PROXY" && \
    wget https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh ${CLANG_VERSION} && \
    rm -f llvm.sh && \
    apt-get update && apt-get install -y --no-install-recommends lld-${CLANG_VERSION} && \
    rm -rf /var/lib/apt/lists/*

# ====================== Install Oh My Posh ======================
RUN curl -s https://ohmyposh.dev/install.sh | bash -s -- -d /usr/local/bin
RUN echo 'eval "$(oh-my-posh init bash --config /root/omp.json)"' >> /root/.bashrc
# =================================================================

# ── source ROS ──
RUN echo "source /opt/ros/lyrical/setup.bash" > /etc/profile.d/ros.sh && chmod +x /etc/profile.d/ros.sh
RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

WORKDIR /workspace
