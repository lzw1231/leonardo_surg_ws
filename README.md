# 环境部署操作说明
## 一、Xpra 远程 GUI（rviz2/gazebo）本机连接步骤
> 前提：远程 192.168.10.23 容器已启动，容器内 Xpra、GUI 程序已运行

1. **SSH 端口转发（保持窗口常驻）**
```bash
ssh -N -L 14501:127.0.0.1:14500 lzw@192.168.10.23
```

2. **Xpra 客户端连接**

```
GDK_BACKEND=x11 xpra attach --username=root tcp://127.0.0.1:14501/
```

## 二、远程主机 Buildx 构建报错修复

> 
> 新环境仅执行一次，永久解决 ROS2 Lyrical Buildx Bake `fs.read` 权限报错

```
echo 'export BUILDX_BAKE_ENTITLEMENTS_FS=0' >> ~/.bashrc
source ~/.bashrc
```