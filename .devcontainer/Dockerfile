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
 && pip3 install --no-cache-dir --break-system-packages ruff \
 && rm -rf /var/lib/apt/lists/*

# ── ROS2 包 ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-ros2-control \
    ros-lyrical-ros2-controllers \
    ros-lyrical-xacro \
 && rm -rf /var/lib/apt/lists/*

# ── 直接 COPY 宿主机上精简好的 LLVM（clang / clang++ / lld / clangd 等）──
COPY llvm-23-minimal /opt/LLVM-23

# ── 让动态链接器认识 /opt/LLVM-23/lib ──
RUN echo "/opt/LLVM-23/lib" > /etc/ld.so.conf.d/llvm23.conf && ldconfig

# ── 验证（不含 lldb 相关）──
RUN /opt/LLVM-23/bin/clang --version | head -1 \
 && /opt/LLVM-23/bin/ld.lld --version | head -1 \
 && /opt/LLVM-23/bin/clangd --version | head -1 \
 && du -sh /opt/LLVM-23

# ── source ROS ──
RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc \
 && echo "source /opt/ros/lyrical/setup.bash" >> /etc/bash.bashrc

# ── 设置彩色终端提示符（对所有用户生效）──
RUN echo "export PS1='\[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]# '" >> /root/.bashrc \
 && echo "export PS1='\[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]# '" >> /etc/bash.bashrc

WORKDIR /workspace
