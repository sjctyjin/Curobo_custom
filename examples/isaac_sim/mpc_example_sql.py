#!/usr/bin/env python3
#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION is strictly prohibited.
#

try:
    # Third Party
    import isaacsim
except ImportError:
    pass

# Third Party
import torch

a = torch.zeros(4, device="cuda:0")

# Standard Library
import argparse
import time
import pyodbc  # Added for SQL database connectivity

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)
parser.add_argument(
    "--visualize_spheres",
    action="store_true",
    help="When True, visualizes robot spheres",
    default=False,
)
parser.add_argument("--robot", type=str, default="franka.yml", help="robot configuration to load")
# 添加WebSocket参数
parser.add_argument("--enable_rosbridge", action="store_true", default=True, help="启用ROSBridge WebSocket发送")
parser.add_argument("--rosbridge_host", type=str, default="192.168.3.125", help="ROSBridge服务器主机")
parser.add_argument("--rosbridge_port", type=int, default=9090, help="ROSBridge服务器端口")
parser.add_argument("--rosbridge_topic", type=str, default="/joint_states", help="ROS关节状态话题")
parser.add_argument("--frame_id", type=str, default="base_link", help="消息头中的frame_id")
parser.add_argument("--print_interval", type=int, default=100, help="打印间隔（步数）")
parser.add_argument("--fix_joint6", action="store_true", default=False, help="将joint6的值固定为0")
# 添加SQL连接参数
parser.add_argument("--enable_sql", action="store_true", default=True, help="启用SQL数据库连接")
parser.add_argument("--sql_server", type=str, default="192.168.3.105", help="SQL Server地址")
parser.add_argument("--sql_database", type=str, default="Fanuc", help="数据库名称")
parser.add_argument("--sql_username", type=str, default="sa", help="数据库用户名")
parser.add_argument("--sql_password", type=str, default="pass", help="数据库密码")
parser.add_argument("--sql_update_interval", type=int, default=10, help="数据库更新间隔（步数）")
args = parser.parse_args()

# ===== SQL连接函数 =====
def connect_sql(server, database, username, password):
    try:
        # 定义连接字符串
        conn = pyodbc.connect(
            f'DRIVER={{ODBC Driver 17 for SQL Server}};'  # 驱动程序名称
            f'SERVER={server};'  # 数据库服务器的 IP 地址或名称
            f'DATABASE={database};'  # 数据库名称
            f'UID={username};'  # 用户名
            f'PWD={password};'  # 密码
        )
        print(f"已成功连接到 SQL Server: {server}, 数据库: {database}")
        return conn
    except Exception as e:
        print(f"连接 SQL 数据库时出错: {e}")
        return None

# ===== 更新SQL数据库中的关节数值 =====
def update_joint_values_in_db(conn, joint_values, joint_names=None):
    try:
        # 将关节值转换为角度
        joint_angles = np.degrees(joint_values)
        
        # 如果提供了关节名称，构建映射
        joint_mapping = {}
        if joint_names is not None:
            for i, name in enumerate(joint_names):
                if i < len(joint_angles):
                    joint_mapping[name] = joint_angles[i]
        
        # 根据关节映射准备SQL更新语句
        # 如果没有映射，则假设顺序为J1-J6
        if joint_mapping:
            # 尝试从映射中获取值，找不到则使用默认值0
            j1 = joint_mapping.get('joint1', 0) if 'joint1' in joint_mapping else joint_mapping.get('J1', 0)
            j2 = joint_mapping.get('joint2', 0) if 'joint2' in joint_mapping else joint_mapping.get('J2', 0)
            j3_raw = joint_mapping.get('joint3', 0) if 'joint3' in joint_mapping else joint_mapping.get('J3', 0)
            j2_raw = joint_mapping.get('joint2', 0) if 'joint2' in joint_mapping else joint_mapping.get('J2', 0)
            j3 = j3_raw - j2_raw  # 根据提供的逻辑计算J3
            j4 = joint_mapping.get('joint4', 0) if 'joint4' in joint_mapping else joint_mapping.get('J4', 0)
            j5 = joint_mapping.get('joint5', 0) if 'joint5' in joint_mapping else joint_mapping.get('J5', 0)
            j6 = joint_mapping.get('joint6', 0) if 'joint6' in joint_mapping else joint_mapping.get('J6', 0)
        else:
            # 默认假设顺序为J1-J6
            j1 = joint_angles[0] if len(joint_angles) > 0 else 0
            j2 = joint_angles[1] if len(joint_angles) > 1 else 0
            j3 = joint_angles[2] - joint_angles[1] if len(joint_angles) > 2 else 0  # 根据提供的逻辑计算J3
            j4 = joint_angles[3] if len(joint_angles) > 3 else 0
            j5 = joint_angles[4] if len(joint_angles) > 4 else 0
            j6 = joint_angles[5] if len(joint_angles) > 5 else 0
        
        # 准备SQL更新语句
        sql = f"""UPDATE PR_Status 
                 SET X_J1 = '{round(j1, 2)}',
                     Y_J2 = '{round(j2, 2)}',
                     Z_J3 = '{round(j3, 2)}',
                     W_J4 = '{round(j4, 2)}',
                     P_J5 = '{round(j5, 2)}',
                     R_J6 = '{round(j6, 2)}',
                     move = '1',
                     moveType = 'joint',
                     time = '{time.strftime("%Y-%m-%d %H:%M:%S")}'
                 WHERE PR_No = 'PR[4]'"""
                 
        # 执行SQL更新
        cursor = conn.cursor()
        cursor.execute(sql)
        conn.commit()
        print(f"已更新数据库中的关节值: J1={round(j1, 2)}, J2={round(j2, 2)}, J3={round(j3, 2)}, J4={round(j4, 2)}, J5={round(j5, 2)}, J6={round(j6, 2)}")
        return True
    except Exception as e:
        print(f"更新SQL数据库时出错: {e}")
        try:
            conn.rollback()
        except:
            pass
        return False

# 如果启用ROSBridge，导入模块
if args.enable_rosbridge:
    try:
        from rosbridge_websocket import init_rosbridge, publish_joint_state, close_rosbridge
        # 初始化ROSBridge客户端
        rosbridge_client = init_rosbridge(args.rosbridge_host, args.rosbridge_port)
        print(f"已初始化ROSBridge客户端，将发送关节值到 {args.rosbridge_topic}")
    except ImportError:
        print("警告: 未找到rosbridge_websocket模块，无法发送关节值")
        args.enable_rosbridge = False

# Third Party
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)

import os
import carb
import numpy as np
from helper import add_extensions, add_robot_to_scene
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
from omni.isaac.core.utils.types import ArticulationAction

# CuRobo
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState as RobotJointState
from curobo.types.state import JointState
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
import inspect

# 导入正向运动学计算所需模块
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from scipy.spatial.transform import Rotation as R

def draw_points(rollouts: torch.Tensor):
    if rollouts is None:
        return
    try:
        from omni.isaac.debug_draw import _debug_draw
    except ImportError:
        from isaacsim.util.debug_draw import _debug_draw
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_points()
    cpu_rollouts = rollouts.cpu().numpy()
    b, h, _ = cpu_rollouts.shape
    point_list = []
    colors = []
    for i in range(b):
        for j in range(h):
            point_list.append(tuple(cpu_rollouts[i, j]))
            colors.append((1.0 - (i + 1.0 / b), 0.3 * (i + 1.0 / b), 0.0, 0.1))
    sizes = [10.0] * (b * h)
    draw.draw_points(point_list, colors, sizes)


def main():
    # 初始化SQL连接
    sql_conn = None
    if args.enable_sql:
        try:
            sql_conn = connect_sql(args.sql_server, args.sql_database, args.sql_username, args.sql_password)
        except Exception as e:
            print(f"初始化SQL连接时出错: {e}")
            print("将继续执行程序，但不会更新SQL数据库")
    
    # 跟踪上一次更新到数据库的关节值
    last_joint_positions = None
    sql_update_step = 0  # 用于跟踪SQL更新间隔
    
    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage
    xform = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(xform)
    stage.DefinePrim("/curobo", "Xform")
    my_world.scene.add_default_ground_plane()

    target = cuboid.VisualCuboid(
        "/World/target",
        position=np.array([0.5, 0, 0.5]),
        orientation=np.array([0, 1, 0, 0]),
        color=np.array([1.0, 0, 0]),
        size=0.05,
    )

    setup_curobo_logger("warn")
    usd_help = UsdHelper()
    tensor_args = TensorDeviceType()

    # 加载机器人配置
    robot_cfg = load_yaml(join_path(get_robot_configs_path(), args.robot))["robot_cfg"]
    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]
    robot_cfg["kinematics"]["collision_sphere_buffer"] += 0.02

    robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)
    articulation_controller = robot.get_articulation_controller()
    # —— 必须在此初始化 controller，否则 _articulation_view/handle 均为 None —— 
    try:
        view = robot._articulation_view
    except AttributeError:
        view = robot._articulation_handle
        
    articulation_controller.initialize(view)

    # 构建 world config
    world_cfg_table = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    )
    world_cfg_table.cuboid[0].pose[2] -= 0.04
    world_cfg1 = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    ).get_mesh_world()
    world_cfg1.mesh[0].name += "_mesh"
    world_cfg1.mesh[0].pose[2] = -10.5
    world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid, mesh=world_cfg1.mesh)

    # MPC 配置
    n_obstacle_cuboids = 30
    n_obstacle_mesh = 10
    print(inspect.signature(MpcSolverConfig.load_from_robot_config))
    mpc_config = MpcSolverConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        use_cuda_graph=True,
        use_cuda_graph_metrics=True,
        use_cuda_graph_full_step=False,
        self_collision_check=True,
        collision_checker_type=CollisionCheckerType.MESH,
        collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
        use_mppi=True,
        use_lbfgs=False,
        use_es=False,
        store_rollouts=True,
        step_dt=0.02,
    )

    
    mpc = MpcSolver(mpc_config)

    # 初始化状态与目标
    retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
    joint_names = mpc.rollout_fn.joint_names
    state = mpc.rollout_fn.compute_kinematics(
        RobotJointState.from_position(retract_cfg, joint_names=joint_names)
    )
    current_state = RobotJointState.from_position(retract_cfg, joint_names=joint_names)
    retract_pose = Pose(state.ee_pos_seq, quaternion=state.ee_quat_seq)
    goal = Goal(
        current_state=current_state,
        goal_state=RobotJointState.from_position(retract_cfg, joint_names=joint_names),
        goal_pose=retract_pose,
    )
    goal_buffer = mpc.setup_solve_single(goal, 1)
    mpc.update_goal(goal_buffer)
    mpc.step(current_state, max_attempts=2)

    # 正向运动学计算 - 根据URDF加载模型
    urdf_file_path = None
    base_link = None
    ee_link = None
    
    # 检查机器人类型，加载相应的URDF
    if "piper" in args.robot.lower():
        urdf_file_path = robot_cfg.get("kinematics", {}).get("urdf_path", None)
        base_link = "link_0"  # 假设基本链接名称
        ee_link = "link_ee"   # 假设末端执行器链接名称
    else:
        # 默认使用Franka或通用配置
        urdf_file_path = robot_cfg.get("kinematics", {}).get("urdf_path", None)
        base_link = "panda_link0"
        ee_link = "panda_link8"
    
    # 尝试初始化运动学模型
    kin_model = None
    if urdf_file_path and os.path.exists(urdf_file_path):
        try:
            FK_robot_cfg = RobotConfig.from_basic(
                urdf_file_path, base_link, ee_link, tensor_args
            )
            kin_model = CudaRobotModel(FK_robot_cfg.kinematics)
            print(f"已加载正向运动学模型，URDF: {urdf_file_path}")
        except Exception as e:
            print(f"加载正向运动学模型时出错: {e}")
            print("将继续执行，但不会计算末端位姿")

    usd_help.load_stage(my_world.stage)
    add_extensions(simulation_app, args.headless_mode)

    past_pose = None
    init_world = False
    cmd_state_full = None
    step = 0

    try:
        while simulation_app.is_running():
            if not init_world:
                for _ in range(10):
                    my_world.step(render=True)
                init_world = True

            draw_points(mpc.get_visual_rollouts())
            my_world.step(render=True)
            if not my_world.is_playing():
                continue

            step += 1
            step_index = step

            # -- 如果需要在 reset 时重绑 --
            if step_index <= 10:
                robot._articulation_view.initialize()
                articulation_controller.initialize(robot._articulation_view)
                idx_list = [robot.get_dof_index(x) for x in j_names]
                robot.set_joint_positions(default_config, idx_list)
                robot._articulation_view.set_max_efforts(
                    values=np.array([5000] * len(idx_list)), joint_indices=idx_list
                )

            if step_index % 1000 == 0:
                print(f"步骤 {step_index}: 更新障碍物...")
                obstacles = usd_help.get_obstacles_from_stage(
                    only_paths=["/World"],
                    ignore_substring=[robot_prim_path, "/World/target", "/World/defaultGroundPlane", "/curobo"],
                    reference_prim_path=robot_prim_path,
                )
                obstacles.add_obstacle(world_cfg_table.cuboid[0])
                mpc.world_coll_checker.load_collision_model(obstacles)

            cube_position, cube_orientation = target.get_world_pose()
            
            if args.robot == "piper":
                cube_orientation = [0.685, 0.0, 0.729, 0.0]
            else:
                cube_orientation =  [-1.0, -0.001, -0.004, 0.0]#[-0.001, 1.000, 0.000, 0.004]
                
            if past_pose is None:
                past_pose = cube_position + 1.0
            if np.linalg.norm(cube_position - past_pose) > 1e-3:
                print(f"目标移动到 {cube_position}, 规划新轨迹...")
                ik_goal = Pose(
                    position=tensor_args.to_device(cube_position),
                    quaternion=tensor_args.to_device(cube_orientation),
                )
                goal_buffer.goal_pose.copy_(ik_goal)
                mpc.update_goal(goal_buffer)
                past_pose = cube_position

            sim_js = robot.get_joints_state()
            if sim_js is None:
                continue
                
            sim_js_names = robot.dof_names

            cu_js = JointState(
                position=tensor_args.to_device(sim_js.positions),
                velocity=torch.zeros_like(tensor_args.to_device(sim_js.velocities)),
                acceleration=torch.zeros_like(tensor_args.to_device(sim_js.velocities)),
                jerk=torch.zeros_like(tensor_args.to_device(sim_js.velocities)),
                joint_names=sim_js_names,
            ).get_ordered_joint_state(mpc.rollout_fn.joint_names)

            if cmd_state_full is None:
                current_state.copy_(cu_js)
            else:
                partial = cmd_state_full.get_ordered_joint_state(mpc.rollout_fn.joint_names)
                current_state.copy_(partial)
                current_state.joint_names = partial.joint_names

            mpc_result = mpc.step(current_state, max_attempts=2)
            cmd_state_full = mpc_result.js_action

            idx_list = [
                robot.get_dof_index(x)
                for x in sim_js_names
                if x in cmd_state_full.joint_names
            ]
           
            common_js = [
                x for x in sim_js_names if x in cmd_state_full.joint_names
            ]
            
            cmd_state = cmd_state_full.get_ordered_joint_state(common_js)
            # 获取关节位置值
            joint_positions = cmd_state.position.view(-1).cpu().numpy()
        
                
            # ===== 通过ROSBridge发送关节值 =====
            if args.enable_rosbridge:
                publish_joint_state(
                    args.rosbridge_topic, 
                    joint_positions, 
                    sim_js_names, 
                    args.frame_id
                )
            
            # ===== 更新SQL数据库 =====
            if args.enable_sql and sql_conn is not None:
                # 根据更新间隔决定是否更新数据库
                sql_update_step += 1
                if sql_update_step >= args.sql_update_interval:
                    # 只在关节值有显著变化或达到更新间隔时更新
                    if (last_joint_positions is None or 
                        np.any(np.abs(joint_positions - last_joint_positions) > 0.01)):
                        try:
                            update_joint_values_in_db(sql_conn, joint_positions, common_js)
                            last_joint_positions = np.copy(joint_positions)
                        except Exception as e:
                            print(f"更新数据库时出错: {e}")
                    sql_update_step = 0
                    
            # ===== 计算并显示末端执行器位姿 =====
            if kin_model is not None and step_index % 100 == 0:
                try:
                    # 确保关节值形状正确
                    q = torch.tensor([joint_positions], device='cuda:0')
                    state = kin_model.get_state(q)
                    position = state.ee_position[0].cpu().numpy()
                    quat = state.ee_quaternion[0].cpu().numpy()
                    
                    # 将四元数转换为欧拉角（以度为单位）
                    # 注意顺序从 [w, x, y, z] 转为 [x, y, z, w]
                    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
                    euler = r.as_euler('zyx', degrees=True)
                    
                    print(f"步骤 {step_index} 末端执行器位姿:")
                    print(f"  - 位置 (xyz): [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]")
                    print(f"  - 姿态 (zyx欧拉角, 度): [{euler[0]:.4f}, {euler[1]:.4f}, {euler[2]:.4f}]")
                except Exception as e:
                    print(f"计算末端执行器位姿时出错: {e}")
            
            # 定期打印关节值
            if step_index % args.print_interval == 0:
                joint_str = ", ".join([f"{joint}: {val:.4f}" for joint, val in zip(sim_js_names, joint_positions)])
                print(f"步骤 {step_index}, 关节位置: {joint_str}")

            art_action = ArticulationAction(
                joint_positions,
                joint_indices=idx_list,
            )
            articulation_controller.apply_action(art_action)

    except KeyboardInterrupt:
        print("收到键盘中断，正在关闭...")
    except Exception as e:
        print(f"主循环中出错: {e}")
    finally:
        # 清理资源
        if args.enable_rosbridge:
            close_rosbridge()
        if args.enable_sql and sql_conn is not None:
            try:
                sql_conn.close()
                print("SQL数据库连接已关闭")
            except:
                pass
        simulation_app.close()
        print("模拟已结束")


if __name__ == "__main__":
    main()