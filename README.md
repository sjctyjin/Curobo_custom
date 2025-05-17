<!--
Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
property and proprietary rights in and to this material, related
documentation and any modifications thereto. Any use, reproduction,
disclosure or distribution of this material and related documentation
without an express license agreement from NVIDIA CORPORATION or
its affiliates is strictly prohibited.
-->
# 本地運行範例 
```bat
1. 建構一個python3.10的虛擬環境
python -m venv venv310

2. 啟動python3.10環境
venv310\scripts\activate

python -m pip install wheel

3. 安裝CUDA 11.8
https://developer.nvidia.com/cuda-11-8-0-download-archive?target_os=Windows&target_arch=x86_64&target_version=11&target_type=exe_local

安裝cudnn
https://developer.nvidia.com/rdp/cudnn-archive

4.在虛擬環境下設定CUDA_HOME(要先完成第三步)

$env:CUDA_HOME="C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.8"
$env:PATH="$env:CUDA_HOME\bin;$env:PATH"
echo $env:CUDA_HOME

5.安裝pytorch 2.2版本
# CUDA 11.8
pip install torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 --index-url https://download.pytorch.org/whl/cu118
安裝isaac sim

pip install isaacsim==4.2.0.2 --extra-index-url https://pypi.nvidia.com
pip install isaacsim-extscache-physics==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2 --extra-index-url https://pypi.nvidia.com

6.確認C++編譯環境是VS2022 MSVC (14.42) v142

開啟Visual Studio 2022/2019 installer

按修改

右邊安裝詳細資料列表中找到 "使用C++的桌面開發"

勾選 "MSVC v142 - VS 2019 C++ x64/x86 建置工具(v14.29)"

後執行安裝。

並在
6. 執行curobo安裝
cd curobo
python -m pip install -e .[isaacsim] --no-build-isolation
如果執行上面那段出錯
先執行 :  pip install wheel 後 在執行上面那段一次
# 執行範例
# IK碰撞球範圍 https://curobo.org/get_started/2b_isaacsim_examples.html

python examples/isaac_sim/ik_reachability.py --robot cr10_ial.yml --visualize_spheres
python examples/isaac_sim/ik_reachability.py --robot piper.yml --visualize_spheres

# motion軌跡規劃 https://curobo.org/get_started/2b_isaacsim_examples.html
python examples/isaac_sim/motion_gen_reacher.py --robot piper.yml --visualize_spheres
python examples/isaac_sim/motion_gen_reacher.py --robot cr10_ial.yml --visualize_spheres

# MPC軌跡規劃 https://curobo.org/get_started/2b_isaacsim_examples.html
python examples/isaac_sim/mpc_example.py --robot piper.yml --visualize_spheres
python examples/isaac_sim/mpc_example.py --robot cr10_ial.yml --visualize_spheres
# MPC軌跡規劃 發布關節值到ROS2主機
python examples/isaac_sim/mpc_example.py --robot piper.yml --enable_rosbridge --rosbridge_host 192.168.3.125

```

# cuRobo

*CUDA Accelerated Robot Library*

**Check [curobo.org](https://curobo.org) for installing and getting started with examples!**

Use [Discussions](https://github.com/NVlabs/curobo/discussions) for questions on using this package.

Use [Issues](https://github.com/NVlabs/curobo/issues) if you find a bug.


cuRobo's collision-free motion planner is available for commercial applications as a
MoveIt plugin: [Isaac ROS cuMotion](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion)

For business inquiries of this python library, please visit our website and submit the form: [NVIDIA Research Licensing](https://www.nvidia.com/en-us/research/inquiries/)


## Overview

cuRobo is a CUDA accelerated library containing a suite of robotics algorithms that run significantly faster than existing implementations leveraging parallel compute. cuRobo currently provides the following algorithms: (1) forward and inverse kinematics,
(2) collision checking between robot and world, with the world represented as Cuboids, Meshes, and Depth images, (3) numerical optimization with gradient descent, L-BFGS, and MPPI, (4) geometric planning, (5) trajectory optimization, (6) motion generation that combines inverse kinematics, geometric planning, and trajectory optimization to generate global motions within 30ms.

<p align="center">
<img width="500" src="images/robot_demo.gif">
</p>


cuRobo performs trajectory optimization across many seeds in parallel to find a solution. cuRobo's trajectory optimization penalizes jerk and accelerations, encouraging smoother and shorter trajectories. Below we compare cuRobo's motion generation on the left to a BiRRT planner for the motion planning phases in a pick and place task.

<p align="center">
<img width="500" src="images/rrt_compare.gif">
</p>


## Citation

If you found this work useful, please cite the below report,

```
@misc{curobo_report23,
      title={cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation},
      author={Balakumar Sundaralingam and Siva Kumar Sastry Hari and Adam Fishman and Caelan Garrett
              and Karl Van Wyk and Valts Blukis and Alexander Millane and Helen Oleynikova and Ankur Handa
              and Fabio Ramos and Nathan Ratliff and Dieter Fox},
      year={2023},
      eprint={2310.17274},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
