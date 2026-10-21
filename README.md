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




A	Aloe（芦荟）  ————   从臂
B	Bamboo（竹）
C	Cactus（仙人掌）
D	Daisy（雏菊）
E	Eucalyptus（桉树）
F	Fern（蕨类）
G	Ginkgo（银杏）
H	Hibiscus（木槿）
I	Iris（鸢尾）
J	Jasmine（茉莉）
K	Kale（羽衣甘蓝）
L	Lavender（薰衣草）
M	Mint（薄荷）
N	Narcissus（水仙）
O	Orchid（兰花）
P	Peony（牡丹）
Q	Quince（榅桲）
R	Rose（玫瑰）
S	Sunflower（向日葵）
T	Tulip（郁金香）
U	Umbrella tree（伞树/鹅掌柴）
V	Violet（紫罗兰）
W	Willow（柳树）
X	Xanthoceras（文冠果）
Y	Yarrow（蓍草）
Z	Zinnia（百日菊）