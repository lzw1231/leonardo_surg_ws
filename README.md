# Xpra 远程 GUI（rviz2/gazebo）本机连接步骤

> 前提：远程 192.168.10.23 容器已启动，容器内 Xpra、GUI 程序已运行

1. 终端 1（SSH 端口转发，保持窗口不要关闭）
ssh -L 14500:127.0.0.1:14500 lzw@192.168.10.23

2. 终端 2（Xpra 客户端连接）
conda activate xpra-client && xpra attach --username=root tcp://127.0.0.1:14500/
