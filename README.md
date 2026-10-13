# 环境部署操作说明
## 一、宿主机显示docker容器内画面的设置
说明: 宿主机每次重启后，在容器启动前，在宿主机上执行 xhost +
```
xhost +
```

## 二、远程主机 Buildx 构建报错修复

说明：新环境仅执行一次，永久解决 ROS2 Lyrical Buildx Bake fs.read 权限报错。

```
echo 'export BUILDX_BAKE_ENTITLEMENTS_FS=0' >> ~/.bashrc
source ~/.bashrc
```