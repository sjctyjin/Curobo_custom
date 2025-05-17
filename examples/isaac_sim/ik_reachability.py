try:
    # Third Party
    import isaacsim
except ImportError:
    pass

# Third Party
import torch
import time

a = torch.zeros(4, device="cuda:0")

# Standard Library
import argparse

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
args = parser.parse_args()

############################################################

# Third Party
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)
# Standard Library
from typing import Dict

# Third Party
import carb
import numpy as np
from helper import add_extensions, add_robot_to_scene
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid, sphere

# CuRobo
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel

# from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import (
    get_assets_path,
    get_filename,
    get_path_of_dir,
    get_robot_configs_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

def get_pose_grid(n_x, n_y, n_z, max_x, max_y, max_z):
    x = np.linspace(-max_x, max_x, n_x)
    y = np.linspace(-max_y, max_y, n_y)
    z = np.linspace(0, max_z, n_z)
    x, y, z = np.meshgrid(x, y, z, indexing="ij")

    position_arr = np.zeros((n_x * n_y * n_z, 3))
    position_arr[:, 0] = x.flatten()
    position_arr[:, 1] = y.flatten()
    position_arr[:, 2] = z.flatten()
    return position_arr


def draw_points(pose, success):
    # Third Party
    try:
        from omni.isaac.debug_draw import _debug_draw
    except ImportError:
        from isaacsim.util.debug_draw import _debug_draw

    draw = _debug_draw.acquire_debug_draw_interface()
    # if draw.get_num_points() > 0:
    draw.clear_points()
    cpu_pos = pose.position.cpu().numpy()
    b, _ = cpu_pos.shape
    point_list = []
    colors = []
    for i in range(b):
        # get list of points:
        point_list += [(cpu_pos[i, 0], cpu_pos[i, 1], cpu_pos[i, 2])]
        if success[i].item():
            colors += [(0, 1, 0, 0.25)]
        else:
            colors += [(1, 0, 0, 0.25)]
    sizes = [40.0 for _ in range(b)]

    draw.draw_points(point_list, colors, sizes)

def debug_coordinate_system(robot, ik_solver):
    """打印机器人坐标系信息进行调试"""
    print("\n====== 调试机器人坐标系 ======")
    
    # 检查正向运动学
    retract_cfg = ik_solver.get_retract_config().view(1, -1)
    fk_state = ik_solver.fk(retract_cfg)
    
    # 打印末端执行器位置
    print(f"末端执行器位置: {fk_state.ee_pose.position.cpu().numpy()}")
    print(f"末端执行器方向: {fk_state.ee_pose.quaternion.cpu().numpy()}")
    
    # 打印关节限制
    joint_limits = ik_solver.kinematics.get_joint_limits()
    print("\n关节限制:")
    for i, name in enumerate(ik_solver.kinematics.joint_names):
        lower = joint_limits[0][0, i].item()
        upper = joint_limits[1][0, i].item()
        print(f"  {name}: [{lower:.3f}, {upper:.3f}]")
    
    # 打印世界定义
    print("\n世界坐标定义:")
    print(f"  机器人基座位置: {robot._articulation_view.get_world_poses()[0]}")
    
    # 生成测试点并检查可达性
    print("\n方向测试点可达性:")
    tensor_args = TensorDeviceType()
    # 测试四个方向
    test_points = [
        [0.5, 0.0, 0.5],   # 前方
        [-0.5, 0.0, 0.5],  # 后方
        [0.0, 0.5, 0.5],   # 右侧
        [0.0, -0.5, 0.5],  # 左侧
        [0.0, 0.0, 0.8],   # 上方
        [0.0, 0.0, 0.2]    # 下方
    ]
    directions = ["前方", "后方", "右侧", "左侧", "上方", "下方"]
    
    # 使用当前姿态
    sim_js = robot.get_joints_state()
    sim_js_names = robot.dof_names
    cu_js = JointState(
        position=tensor_args.to_device(sim_js.positions),
        velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
        acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
        jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
        joint_names=sim_js_names,
    )
    
    for i, point in enumerate(test_points):
        # 创建目标姿势
        test_pose = Pose(
            position=tensor_args.to_device([point]),
            quaternion=tensor_args.to_device([[0, 1, 0, 0]]),  # 保持与原来相同的方向
        )
        
        # 尝试解算
        result = ik_solver.solve_batch(test_pose)
        success = result.success.item()
        print(f"  {directions[i]} ({point}): {'可达' if success else '不可达'}")
    
    print("====== 调试结束 ======\n")

def main():
    try:
        # 设置基本日志级别
        setup_curobo_logger("warn")
        
        # 首先加载扩展，确保在世界创建前完成
        add_extensions(simulation_app, args.headless_mode)
        
        # 创建世界和场景
        my_world = World(stage_units_in_meters=1.0)
        stage = my_world.stage

        xform = stage.DefinePrim("/World", "Xform")
        stage.SetDefaultPrim(xform)
        stage.DefinePrim("/curobo", "Xform")

        # 添加目标立方体
        target = cuboid.VisualCuboid(
            "/World/target",
            position=np.array([0.5, 0, 0.5]),
            orientation=np.array([0, 1, 0, 0]),
            color=np.array([1.0, 0, 0]),
            size=0.05,
        )

        # 初始化变量
        past_pose = None
        target_pose = None
        n_obstacle_cuboids = 30
        n_obstacle_mesh = 10
        
        # 创建USD帮助器
        usd_help = UsdHelper()
        usd_help.load_stage(my_world.stage)

        # 创建张量参数
        tensor_args = TensorDeviceType()

        # 加载机器人配置
        robot_cfg = load_yaml(join_path(get_robot_configs_path(), args.robot))["robot_cfg"]
        j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
        default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]

        # 加载世界配置
        world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
        )
        world_cfg_table.cuboid[0].pose[2] -= 0.002
        world_cfg1 = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
        ).get_mesh_world()
        world_cfg1.mesh[0].name += "_mesh"
        world_cfg1.mesh[0].pose[2] = -10.5

        world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid, mesh=world_cfg1.mesh)

        # 将世界添加到场景
        usd_help.add_world_to_stage(world_cfg, base_frame="/World")
        my_world.scene.add_default_ground_plane()

        # 添加机器人到场景
        robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)
        
        # 初始化物理引擎
        my_world.reset()
        # 运行几步确保物理初始化
        for _ in range(5):
            my_world.step(render=True)

        print("正在初始化机器人...")
        # 在初始化IK求解器前，先确保机器人初始姿态设置正确
        try:
            # 确保ArticulationView已初始化
            if hasattr(robot, "_articulation_view"):
                robot._articulation_view.initialize()
                
            # 设置初始姿态
            idx_list = [robot.get_dof_index(x) for x in j_names]
            robot.set_joint_positions(default_config, idx_list)
            
            # 设置最大力矩
            if hasattr(robot._articulation_view, "set_max_efforts"):
                robot._articulation_view.set_max_efforts(
                    values=np.array([5000 for i in range(len(idx_list))]), 
                    joint_indices=idx_list
                )
                
            # 再次运行几帧确保初始状态已设置
            for _ in range(10):
                my_world.step(render=True)
                
        except Exception as e:
            print(f"初始化机器人时出错: {e}")

        # 初始化关节控制器
        articulation_controller = None
        try:
            articulation_controller = robot.get_articulation_controller()
        except Exception as e:
            print(f"获取关节控制器时出错: {e}")

        # 创建IK求解器
        print("初始化IK求解器...")
        ik_config = IKSolverConfig.load_from_robot_config(
            robot_cfg,
            None,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=False,
            self_collision_opt=False,
            tensor_args=tensor_args,
            use_cuda_graph=True,
         
        )
        ik_solver = IKSolver(ik_config)

        # 创建姿态网格
        position_grid_offset = tensor_args.to_device(get_pose_grid(10, 10, 5, 0.5, 0.5, -0.5))


        # 执行正向运动学获取目标姿态
        fk_state = ik_solver.fk(ik_solver.get_retract_config().view(1, -1))
        goal_pose = fk_state.ee_pose
        goal_pose = goal_pose.repeat(position_grid_offset.shape[0])
        goal_pose.position += position_grid_offset
        print("goal_pose : ",goal_pose  )

        # 预热IK求解器
        print("预热IK求解器...")
        result = ik_solver.solve_batch(goal_pose)
        print("CuRobo已准备就绪")

        # 初始化变量
        cmd_plan = None
        cmd_idx = 0
        i = 0
        spheres = None
        physics_initialized = False
        initialization_attempts = 0
        max_initialization_attempts = 50

        # 主循环
        while simulation_app.is_running():
            try:
                my_world.step(render=True)
                
                if not my_world.is_playing():
                    if i % 100 == 0:
                        print("**** 点击Play开始模拟 *****")
                    i += 1
                    continue

                step_index = my_world.current_time_step_index
                
                # 初始化阶段
                if step_index <= 20:
                    try:
                        # 确保ArticulationView已初始化
                        if hasattr(robot, "_articulation_view"):
                            robot._articulation_view.initialize()
                            
                        # 设置初始姿态
                        idx_list = [robot.get_dof_index(x) for x in j_names]
                        robot.set_joint_positions(default_config, idx_list)
                        
                        # 设置最大力矩
                        if hasattr(robot._articulation_view, "set_max_efforts"):
                            robot._articulation_view.set_max_efforts(
                                values=np.array([5000 for i in range(len(idx_list))]), 
                                joint_indices=idx_list
                            )
                            
                    except Exception as e:
                        print(f"初始化关节时出错: {e}")
                    continue

                # 在特定帧更新世界
                if step_index == 50 or step_index % 500 == 0.0:
                    print("更新世界，参考路径:", robot_prim_path)
                    try:
                        obstacles = usd_help.get_obstacles_from_stage(
                            reference_prim_path=robot_prim_path,
                            ignore_substring=[
                                robot_prim_path,
                                "/World/target",
                                "/World/defaultGroundPlane",
                                "/curobo",
                            ],
                        ).get_collision_check_world()
                        print([x.name for x in obstacles.objects])
                        ik_solver.update_world(obstacles)
                        print("世界已更新")
                        carb.log_info("已从场景同步CuRobo世界。")
                    except Exception as e:
                        print(f"更新世界时出错: {e}")

                # 获取目标立方体的位置和方向
                cube_position, cube_orientation = target.get_world_pose()

                if past_pose is None:
                    past_pose = cube_position
                if target_pose is None:
                    target_pose = cube_position

                # 获取关节状态
                sim_js = None
                try:
                    sim_js = robot.get_joints_state()
                except Exception as e:
                    # 仅在初始化阶段频繁打印错误
                    if not physics_initialized and initialization_attempts % 10 == 0:
                        print(f"获取关节状态时出错: {e}")
                
                # 如果关节状态为None，尝试进行恢复
                if sim_js is None:
                    initialization_attempts += 1
                    if initialization_attempts % 10 == 0:
                        print(f"物理引擎尚未初始化，尝试次数: {initialization_attempts}/{max_initialization_attempts}")
                    
                    # 如果尝试多次后仍未成功，尝试重置机器人
                    if initialization_attempts >= max_initialization_attempts:
                        print("尝试重置物理引擎...")
                        try:
                            my_world.reset()
                            # 重新设置关节位置
                            if hasattr(robot, "_articulation_view"):
                                robot._articulation_view.initialize()
                            idx_list = [robot.get_dof_index(x) for x in j_names]
                            robot.set_joint_positions(default_config, idx_list)
                            if hasattr(robot._articulation_view, "set_max_efforts"):
                                robot._articulation_view.set_max_efforts(
                                    values=np.array([5000 for i in range(len(idx_list))]), 
                                    joint_indices=idx_list
                                )
                            # 重置计数器
                            initialization_attempts = 0
                        except Exception as e:
                            print(f"重置物理引擎时出错: {e}")
                    
                    # 等待一下再尝试
                    time.sleep(0.1)
                    continue
                else:
                    # 成功获取关节状态
                    if not physics_initialized:
                        print("物理引擎已成功初始化")
                        physics_initialized = True
                
                # 处理关节状态数据
                sim_js_names = robot.dof_names  # 从机器人对象获取关节名称
                cu_js = JointState(
                    position=tensor_args.to_device(sim_js.positions),
                    velocity=tensor_args.to_device(sim_js.velocities),
                    acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
                    jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
                    joint_names=sim_js_names,  # 使用从robot对象获取的dof_names
                )
                cu_js = cu_js.get_ordered_joint_state(ik_solver.kinematics.joint_names)

                # 可视化球体（如果启用）
                if args.visualize_spheres and step_index % 2 == 0:
                    try:
                        sph_list = ik_solver.kinematics.get_robot_as_spheres(cu_js.position)

                        if spheres is None:
                            spheres = []
                            # 创建球体
                            for si, s in enumerate(sph_list[0]):
                                sp = sphere.VisualSphere(
                                    prim_path="/curobo/robot_sphere_" + str(si),
                                    position=np.ravel(s.position),
                                    radius=float(s.radius),
                                    color=np.array([0, 0.8, 0.2]),
                                )
                                spheres.append(sp)
                        else:
                            for si, s in enumerate(sph_list[0]):
                                spheres[si].set_world_pose(position=np.ravel(s.position))
                                spheres[si].set_radius(float(s.radius))
                    except Exception as e:
                        print(f"可视化球体时出错: {e}")

                # 当目标移动且机器人当前静止时，执行IK
                if (
                    np.linalg.norm(cube_position - target_pose) > 1e-3
                    and np.linalg.norm(past_pose - cube_position) == 0.0
                    and np.linalg.norm(sim_js.velocities) < 0.2
                ):
                    try:
                        # 设置末端执行器目标

                        ee_translation_goal = cube_position

                        ee_orientation_teleop_goal = [ 0.459, 0.523, 0.568, -0.440]#cube_orientation

                        print("末端由拉腳 : ",ee_orientation_teleop_goal)

                        # 计算CuRobo解决方案
                        ik_goal = Pose(
                            position=tensor_args.to_device(ee_translation_goal),
                            quaternion=tensor_args.to_device(ee_orientation_teleop_goal),
                        )
                        goal_pose.position[:] = ik_goal.position[:] + position_grid_offset
                        goal_pose.quaternion[:] = ik_goal.quaternion[:]
                        result = ik_solver.solve_batch(goal_pose)
                        
                        succ = torch.any(result.success)
                        print(
                            "IK完成: 姿态数量: "
                            + str(goal_pose.batch)
                            + " 耗时(秒): "
                            + str(result.solve_time)
                        )
                        # 绘制点和成功标志
                        draw_points(goal_pose, result.success)

                        if succ:
                            # 获取所有解决方案
                            cmd_plan = result.js_solution[result.success]
                            # 获取共同的关节名称
                            idx_list = []
                            common_js_names = []
                            # debug_coordinate_system(robot, ik_solver)

                            for x in sim_js_names:
                                if x in cmd_plan.joint_names:
                                    idx_list.append(robot.get_dof_index(x))
                                    common_js_names.append(x)

                            cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)
                            cmd_idx = 0
                        else:
                            carb.log_warn("规划未收敛到解决方案。不采取任何行动。")
                        target_pose = cube_position
                    except Exception as e:
                        print(f"执行IK时出错: {e}")
                
                past_pose = cube_position
                
                # 执行计划动作
                if cmd_plan is not None and step_index % 20 == 0:
                    try:
                        cmd_state = cmd_plan[cmd_idx]
                        robot.set_joint_positions(cmd_state.position.cpu().numpy(), idx_list)
                        cmd_idx += 1
                        if cmd_idx >= len(cmd_plan.position):
                            cmd_idx = 0
                            cmd_plan = None
                            my_world.step(render=True)
                            robot.set_joint_positions(default_config, idx_list)
                    except Exception as e:
                        print(f"执行计划动作时出错: {e}")
            
            except Exception as e:
                print(f"主循环中出错: {e}")
                
        simulation_app.close()
    
    except Exception as e:
        print(f"程序执行过程中出现错误: {e}")
        simulation_app.close()


if __name__ == "__main__":
    main()