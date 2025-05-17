try:
    import isaacsim
except ImportError:
    pass

import torch
import argparse
import os
import time

from omni.isaac.kit import SimulationApp

# 初始化解析參數
parser = argparse.ArgumentParser()
parser.add_argument("--headless_mode", type=str, default=None)
parser.add_argument("--robot", type=str, default="franka.yml")
parser.add_argument("--external_asset_path", type=str, default=None)
parser.add_argument("--external_robot_configs_path", type=str, default=None)
parser.add_argument("--visualize_spheres", action="store_true", default=False)
parser.add_argument("--reactive", action="store_true", default=False)
parser.add_argument("--constrain_grasp_approach", action="store_true", default=False)
parser.add_argument("--reach_partial_pose", nargs=6, type=float, default=None)
parser.add_argument("--hold_partial_pose", nargs=6, type=float, default=None)
# 添加ROSBridge参数，与mpc_example.py保持一致
parser.add_argument("--enable_rosbridge", action="store_true", default=False, help="启用ROSBridge WebSocket发送")
parser.add_argument("--rosbridge_host", type=str, default="192.168.3.125", help="ROSBridge服务器主机")
parser.add_argument("--rosbridge_port", type=int, default=9090, help="ROSBridge服务器端口")
parser.add_argument("--rosbridge_topic", type=str, default="/joint_custom_state", help="ROS关节状态话题")
parser.add_argument("--frame_id", type=str, default="piper_single", help="消息头中的frame_id")
parser.add_argument("--print_interval", type=int, default=100, help="打印间隔（步数）")
args = parser.parse_args()

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

simulation_app = SimulationApp({"headless": args.headless_mode is not None, "width": "1920", "height": "1080"})

import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid, sphere
from omni.isaac.core.utils.types import ArticulationAction

from helper import add_extensions, add_robot_to_scene
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.geom.types import WorldConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig, PoseCostMetric
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState

# ===== 工具函數 =====
def safe_join_path(path1, path2):
    if isinstance(path2, dict):
        raise ValueError("join_path的第二個參數不能是dict! 應該是檔名字串。")
    return os.path.normpath(os.path.join(path1, path2))

# ===== 主程式 =====
def main():
    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage
    xform = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(xform)
    stage.DefinePrim("/curobo", "Xform")

    target = cuboid.VisualCuboid("/World/target", position=np.array([0.5, 0, 0.5]), orientation=np.array([ 0.459, 0.523, 0.568, -0.440]), color=np.array([1,0,0]), size=0.05)

    setup_curobo_logger("warn")
    usd_help = UsdHelper()

    tensor_args = TensorDeviceType()

    robot_cfg_path = args.external_robot_configs_path or get_robot_configs_path()
    robot_cfg = load_yaml(safe_join_path(robot_cfg_path, args.robot))
    if "robot_cfg" in robot_cfg:
        robot_cfg = robot_cfg["robot_cfg"]

    if args.external_asset_path:
        robot_cfg["kinematics"]["external_asset_path"] = args.external_asset_path

    # 添加扩展，确保在机器人加载前完成
    add_extensions(simulation_app, args.headless_mode)
    
    # 加载世界配置
    world_cfg_table = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), "collision_table.yml")))
    world_cfg_table.cuboid[0].pose[2] -= 0.02
    world_cfg_mesh = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))).get_mesh_world()
    world_cfg_mesh.mesh[0].pose[2] = -10.5

    world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid, mesh=world_cfg_mesh.mesh)

    # 设置USD帮助器
    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg, base_frame="/World")
    my_world.scene.add_default_ground_plane()
    
    # 添加机器人到场景
    robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)
    
    # 确保物理引擎完全启动
    my_world.reset()
    my_world.step(render=True)
    
    # 初始化运动生成器
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        collision_checker_type=CollisionCheckerType.MESH,
        num_trajopt_seeds=12, num_graph_seeds=12
    )
    motion_gen = MotionGen(motion_gen_config)

    print("warming up...")
    motion_gen.warmup(enable_graph=True)
    print("Curobo is Ready")

    articulation_controller = None
    past_pose = None
    cmd_plan = None
    cmd_idx = 0
    i = 0
    step_counter = 0  # 添加步数计数器用于ROSBridge打印
    
    # 等待物理引擎完全初始化
    for _ in range(5):
        my_world.step(render=True)
    
    # 初始化关节控制器
    try:
        articulation_controller = robot.get_articulation_controller()
    except Exception as e:
        print(f"初始化关节控制器出错: {e}")
        articulation_controller = None

    # 初始化机器人姿态
    idx_list = None
    try:
        if robot and hasattr(robot, "_articulation_view"):
            robot._articulation_view.initialize()
            idx_list = [robot.get_dof_index(x) for x in robot_cfg["kinematics"]["cspace"]["joint_names"]]
            robot.set_joint_positions(robot_cfg["kinematics"]["cspace"]["retract_config"], idx_list)
            if hasattr(robot._articulation_view, "set_max_efforts"):
                robot._articulation_view.set_max_efforts(values=np.array([5000] * len(idx_list)), joint_indices=idx_list)
    except Exception as e:
        print(f"初始化机器人姿态出错: {e}")
    
    # 特殊处理Piper机器人，确保joint8的处理正确
    if args.robot == "piper.yml":
        # 记录修改前的关节名称
        original_dof_names = robot.dof_names.copy() if hasattr(robot.dof_names, 'copy') else list(robot.dof_names)
        print(f"原始关节名称: {original_dof_names}")
        
        # 移除joint8（如果存在）
        if 'joint8' in robot.dof_names:
            robot.dof_names.pop()
            print(f"移除joint8后的关节名称: {robot.dof_names}")
    
    # 主循环
    try:
        while simulation_app.is_running():
            try:
                my_world.step(render=True)
                step_counter += 1
                
                if not my_world.is_playing():
                    if i % 100 == 0:
                        print("**** Click Play to start simulation *****")
                    i += 1
                    continue

                step_index = my_world.current_time_step_index

                # 确保控制器已初始化
                if articulation_controller is None and step_index > 10:
                    try:
                        articulation_controller = robot.get_articulation_controller()
                    except Exception as e:
                        print(f"尝试获取关节控制器时出错: {e}")
                        continue

                # 前几帧用于稳定模拟
                if step_index < 20:
                    try:
                        if robot and hasattr(robot, "_articulation_view"):
                            robot._articulation_view.initialize()
                            if idx_list is None:
                                idx_list = [robot.get_dof_index(x) for x in robot_cfg["kinematics"]["cspace"]["joint_names"]]
                            robot.set_joint_positions(robot_cfg["kinematics"]["cspace"]["retract_config"], idx_list)
                            if hasattr(robot._articulation_view, "set_max_efforts"):
                                robot._articulation_view.set_max_efforts(values=np.array([5000] * len(idx_list)), joint_indices=idx_list)
                    except Exception as e:
                        print(f"设置初始姿态时出错: {e}")
                    continue

                # 尝试获取关节状态
                sim_js = None
                try:
                    sim_js = robot.get_joints_state()
                except Exception as e:
                    print(f"获取关节状态出错: {e}")
                    
                if sim_js is None:
                    print("sim_js is None, waiting for physics to initialize...")
                    # 等待一下再尝试
                    time.sleep(0.1)
                    continue
                
                # 处理关节状态
                cu_js = JointState(
                    position=tensor_args.to_device(sim_js.positions),
                    velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
                    acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
                    jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
                    joint_names=robot.dof_names
                )
                cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)
                

                # 获取目标立方体位置
                cube_position, cube_orientation = target.get_world_pose()
                
                if past_pose is None:
                    past_pose = cube_position

                # 如果目标位置变化，计划新路径
                if np.linalg.norm(cube_position - past_pose) > 1e-3:
                    ik_goal = Pose(position=tensor_args.to_device(cube_position), quaternion=tensor_args.to_device(cube_orientation))
                    plan_config = MotionGenPlanConfig(enable_graph=True, max_attempts=20)
                    result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)

                    if result.success.item():
                        cmd_plan = result.get_interpolated_plan()
                        cmd_plan = motion_gen.get_full_js(cmd_plan)
                        idx_list = [robot.get_dof_index(x) for x in robot.dof_names if x in cmd_plan.joint_names]
                        cmd_plan = cmd_plan.get_ordered_joint_state(robot.dof_names)
                        cmd_idx = 0

                    past_pose = cube_position

                # 执行计划的动作
                if cmd_plan is not None and cmd_idx < len(cmd_plan.position) and articulation_controller is not None:
                    try:
                        cmd_state = cmd_plan[cmd_idx]
                        joint_positions = cmd_state.position.cpu().numpy()
                        
                        # 准备用于发送到ROS的关节位置
                        ros_joint_positions = joint_positions.copy()
                        
                        # 如果是piper机器人，需要添加joint8的位置值（仅用于ROSBridge发送）
                        if args.robot == "piper.yml":
                            # 为joint8添加一个默认值（根据mpc_example.py中的处理方式）
                            ros_joint_positions = np.append(ros_joint_positions, -0.35)
                        
                        # 使用ROSBridge发送关节状态
                        if args.enable_rosbridge:
                            ros_joint_names = robot.dof_names + (['joint8'] if args.robot == "piper.yml" else [])
                            publish_joint_state(
                                args.rosbridge_topic,
                                ros_joint_positions,
                                ros_joint_names,
                                args.frame_id
                            )
                            
                            # 定期打印关节值
                            if step_counter % args.print_interval == 0:
                                joint_str = ", ".join([f"{joint}: {val:.4f}" for joint, val in zip(ros_joint_names, ros_joint_positions)])
                                print(f"步骤 {step_counter}, 关节位置: {joint_str}")
                        
                        # 使用原始关节位置(不包含添加的joint8)控制机器人
                        art_action = ArticulationAction(joint_positions, joint_indices=idx_list)
                        articulation_controller.apply_action(art_action)
                        cmd_idx += 1
                    except Exception as e:
                        print(f"应用关节动作时出错: {e}")
                    
            except Exception as e:
                print(f"主循环中出错: {e}")
    
    except KeyboardInterrupt:
        print("收到键盘中断，正在关闭...")
    finally:
        # 关闭ROSBridge连接
        if args.enable_rosbridge:
            close_rosbridge()
        simulation_app.close()
        print("模拟已结束")

if __name__ == "__main__":
    main()