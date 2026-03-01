# YOLOv5 + ORB-SLAM3 路面动态场景视觉定位

自动驾驶和机器人导航中，路面上的行人、车辆等动态障碍物会严重干扰视觉 SLAM 的定位精度——动态物体的特征点在帧间不断变化，导致位姿估计漂移甚至丢失。本项目将 YOLOv5 目标检测与 ORB-SLAM3 视觉定位系统结合，先用 YOLOv5 识别并剔除动态物体区域的特征点，再用 ORB-SLAM3 进行稠密双目重建和定位，从而在车流、人流密集的道路场景中也能保持稳定的定位精度。

## 痛点与目的

- **问题**：传统视觉 SLAM（如 ORB-SLAM3）在静态环境下表现优秀，但一旦场景中出现大量运动物体（行人穿行、车辆行驶），特征匹配会被动态物体"污染"，导致定位精度急剧下降
- **方案**：在 SLAM 前端加入 YOLOv5 语义分割/检测，实时标记画面中的动态物体区域，在特征提取阶段过滤掉这些区域内的特征点，只保留静态背景的特征进行位姿求解
- **效果**：在交通路面等高动态场景中，定位鲁棒性和精度显著提升

## 系统组成

```
双目相机输入
    ↓
YOLOv5 目标检测 → 标记动态目标区域（行人、车辆等）
    ↓
ORB-SLAM3 前端 → 剔除动态区域特征点 → ORB特征提取和匹配
    ↓
ORB-SLAM3 后端优化 + 回环检测
    ↓
稠密双目重建（ELAS 立体匹配）
    ↓
输出：相机轨迹 + 三维稠密点云
```

## 项目结构

```
.
├── YOLOv5_ORBSLAM3_dense_stereo/  # 主项目
│   ├── src/                        # ORB-SLAM3 核心源码
│   ├── include/                    # 头文件
│   ├── Examples/                   # 运行示例
│   ├── ThirdParty/                 # 第三方依赖（DBoW2, g2o, Sophus）
│   ├── Vocabulary/                 # ORB词袋
│   ├── lib/                        # 编译库文件
│   ├── build.sh                    # 编译脚本
│   ├── build_ros.sh                # ROS编译脚本
│   └── CMakeLists.txt              # CMake配置
└── elas-ros_ws/                    # ELAS 立体匹配 ROS 工作空间
    ├── src/                        # ROS 包源码
    ├── build/                      # 编译输出
    └── devel/                      # ROS 开发环境
```

## 编译运行

### 依赖

- OpenCV 4.x
- Eigen3
- Pangolin（可视化）
- libtorch（PyTorch C++ API）
- ROS（可选，用于 ELAS 稠密重建）

### 编译

```bash
cd YOLOv5_ORBSLAM3_dense_stereo
chmod +x build.sh
./build.sh
```

## 适用场景

- 自动驾驶视觉定位
- 室外机器人导航（高动态环境）
- 三维场景稠密重建
- 视觉 SLAM 学术研究

## 技术栈

- ORB-SLAM3（C++）
- YOLOv5 (libtorch)
- ELAS 立体匹配
- ROS
- OpenCV、Eigen3、Pangolin

## License

GPLv3 License
