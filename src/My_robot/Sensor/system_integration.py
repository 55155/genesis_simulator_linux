import argparse
import os
import numpy as np  
from tqdm import tqdm

import roma
import torch

# 오일러 각을 회전 행렬로 변환
euler_angles = [90, 0, 90]  # degrees
R = roma.euler_to_rotmat('XYZ', euler_angles, degrees=True)

# # Rigid 변환 생성
# T = roma.Rigid(linear=R, translation=torch.zeros(3))

# # 포인트 변환
# points = torch.tensor([[1.0, 1.0, 1.0]])
# transformed_points = T[None].apply(points)

# print(transformed_points)



import genesis as gs
from genesis.recorders.plotters import IS_MATPLOTLIB_AVAILABLE, IS_PYQTGRAPH_AVAILABLE

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-dt", "--timestep", type=float, default=0.01, help="Simulation time step")
    parser.add_argument("-v", "--vis", action="store_true", default=True, help="Show visualization GUI")
    parser.add_argument("-nv", "--no-vis", action="store_false", dest="vis", help="Disable visualization GUI")
    parser.add_argument("-c", "--cpu", action="store_true", help="Use CPU instead of GPU")
    parser.add_argument("-t", "--seconds", type=float, default=2.0, help="Number of seconds to simulate")
    parser.add_argument("-f", "--force", action="store_true", default=True, help="Use ContactForceSensor (xyz float)")
    parser.add_argument("-nf", "--no-force", action="store_false", dest="force", help="Use ContactSensor (boolean)")

    args = parser.parse_args()

    ########################## init ##########################
    gs.init(backend=gs.metal)
    # gs.init(backend=gs.cpu if args.cpu else gs.gpu, logging_level=None)

    ########################## scene setup ##########################
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=args.timestep,
        ),
        rigid_options=gs.options.RigidOptions(
            constraint_timeconst=max(0.01, 2 * args.timestep),
            use_gjk_collision=True,
        ),
        vis_options=gs.options.VisOptions(
            show_world_frame=True,
        ),
        viewer_options=gs.options.ViewerOptions(
            max_FPS=20,
        ),
        profiling_options=gs.options.ProfilingOptions(
            show_FPS=True,
        ),
        show_viewer=args.vis,
    )
    cam = scene.add_camera(
        res=(1280, 960),
        pos = (3,0,3),
        lookat=(0, 0, 0.0),
        fov=30,
        
        GUI=True,
    )

    # rigid solver : for add_constaraints
    solver = scene.sim.rigid_solver

    # add entities
    scene.add_entity(gs.morphs.Plane())

    # Crank-slider system
    Crank_slider_system = scene.add_entity(
        gs.morphs.MJCF(
            file = "My_asset/Crank_slider_system_V3_Pjoint_Posmod_description/urdf/" \
            "Crank_slider_system_V3_Pjoint_Posmod.xml",
            pos = (0, 0.0, 0),
            scale = 10.0,
        ),
        surface=gs.surfaces.Default(
            smooth=False,
        ),
    )

    link_name = [
    "motor_shaft_1",
    "Link2_1",
    "Link3_1",
    "Shaft_1",
    "Wall_1",
    ]
    links = [Crank_slider_system.get_link(name) for name in link_name]
    link_idx = {link_name[i]: [None, None] for i in range(len(link_name))}

    # 전역 0, 지역 1
    for i, name in enumerate(link_name):
        link_idx[name][0] = links[i].idx
        link_idx[name][1] = links[i].idx_local

    # Crank-slider system Joint index
    jnt_names = [
        'Revolute 47',
        'Revolute 49',
        'Revolute 50',
        'Slider 61',
    ]
    dofs_idx = [Crank_slider_system.get_joint(name).dof_idx_local for name in jnt_names]

    tablet_link_name = ("Tablet", "segment")
    tablet = scene.add_entity(
        gs.morphs.MJCF(
            file = "My_asset/Tablet_posmod/Tablet_posmod.xml",
            # Crank_slider_system, Wall position = 0.353 0.01 -0.22
            # euler = (90,0,90),
            pos = (-0.48, 3.36, 5),
            scale = 10.0,
        )
    )

    # Tablet link
    tablet_links = [tablet.get_link(name) for name in tablet_link_name]
    tablet_link_idx = {tablet_link_name[i]: [None, None] for i in range(len(tablet_link_name))}
    for i, name in enumerate(tablet_link_name):
        tablet_link_idx[name][0] = tablet_links[i].idx
        tablet_link_idx[name][1] = tablet_links[i].idx_local
    print(tablet_link_idx)

    # add sensors to the scene
    for link_name in tablet_link_name:
        if args.force:
            sensor_options = gs.sensors.ContactForce(
                entity_idx=tablet.idx,
                link_idx_local=tablet.get_link(link_name).idx_local,
                draw_debug=True,
            )
            plot_kwargs = dict(
                title=f"{link_name} Force Sensor Data",
                labels=["force_x", "force_y", "force_z"],
            )
        else:
            sensor_options = gs.sensors.Contact(
                entity_idx=tablet.idx,
                link_idx_local=tablet.get_link(link_name).idx_local,
                draw_debug=True,
            )
            plot_kwargs = dict(
                title=f"{link_name} Contact Sensor Data",
                labels=["in_contact"],
            )

    sensor = scene.add_sensor(sensor_options)

    if IS_PYQTGRAPH_AVAILABLE:
        sensor.start_recording(gs.recorders.PyQtLinePlot(**plot_kwargs))
    elif IS_MATPLOTLIB_AVAILABLE:
        print("pyqtgraph not found, falling back to matplotlib.")
        sensor.start_recording(gs.recorders.MPLLinePlot(**plot_kwargs))
    else:
        print("matplotlib or pyqtgraph not found, skipping real-time plotting.")

    ## scene build
    scene.build()
    print("------------------- Scene Built ------------------")
    print("Scene Enttities : ", scene.entities)    

    # # Equality constraint
    # link1 = tablet.get_link(tablet_link_name[0])
    # link2 = tablet.get_link(tablet_link_name[1])
    # link1_idx_arr = np.array(link1.idx, dtype=gs.np_int)
    # link2_idx_arr = np.array(link2.idx, dtype=gs.np_int)
    # solver.add_weld_constraint(link1_idx_arr, link2_idx_arr)

    # Wall position global : (-0.4435, 0.1300, 0.0500), local tablet_pos = (-0.3, 0.0, 0.0)
    # Wall position == Tablet position
    # tablet position = 0.353 0.01 -0.22 -> 90 0 90 euler

    tablet_pos = Crank_slider_system.get_links_pos(link_idx["Wall_1"][0])
    tablet_pos = tablet_pos.tolist()
    tablet_pos[0][2] += 0.05  # Wall 의 두께 고려
    print(tablet_pos)
    # tablet.set_pos(pos = tablet_pos[0])

    # 특정 link 의 좌표를 가져올 수 있는 게 아닌, 전체 Entity 의 좌표를 가져오는 것임.
    print("Wall_position : ", Crank_slider_system.get_links_pos())
    print("Tablet_position : ", tablet.get_links_pos(), tablet.get_pos())
    cam.start_recording()

    try:
        # second: 10, timestep = 0.01
        steps = int(args.seconds / args.timestep) if "PYTEST_VERSION" not in os.environ else 10
        print("steps : ", steps)
        for _ in tqdm(range(steps)):
            # 일부 노이즈 발생. 
            # print(sensor.read())
            cam.render()
            scene.step()
    except KeyboardInterrupt:
        gs.logger.info("Simulation interrupted, exiting.")
    finally:
        gs.logger.info("Simulation finished.")
        cam.stop_recording(save_to_filename ="video/SystemIntegration_Debug2_posmod_20251014.mp4")
        scene.stop_recording()

if __name__ == "__main__":
    main()


# Wall 의 시뮬레이션 상 좌표 : 0.353 0.01 -0.22 .. ?
# Wall_position :  tensor
#       ([[ 0.0000,  0.0000,  0.0000],
#         [-0.1630,  0.1100,  0.0500],
#         [-0.1485,  0.2555,  0.0500],
#         [ 0.0000,  0.0000,  0.0000],
#         [-0.1435,  0.1300,  0.0500],
#         [-0.1435,  0.2240,  0.0500]], device='cuda:0')
# Tablet_position :  tensor
#        ([[0., 0., 0.],
#         [0., 0., 0.]], device='cuda:0')
#
#  [[-0.14350000023841858, 0.12999999523162842, 0.05000000074505806]]


# 2025.10.14 수정 사항
# 크기가 너무 커지거나 작아지면, 시각화 실패하는 줄 알았는데, 아님. 
# 충돌이 발생하면, 시각화 화면이 black out 되는 듯 함.
# fusion 360 기준으로 포지션 지정하는 게 좋음. 무슨 말이냐면, fusion 360의 오리진을 무조건 따라감.
#   - 예를 들어서 (0, 0, 0) 을 기준으로 만들지 않으면 pos 지정이 애매해짐.


# 2025.10.15 해야할것. 
# pyLife 알아보기 -> SN 선도 근사할 아이디어 생각해보기
# 위치는 대충 조정 된듯함. motor_shaft_1 의 각도 조정 dof_idx_position ,..? 이런 함수. 

# Tablet posmod freejoint 제거하고, Wall 위치에 맞게 pos 조정하기. -> 완료
# Tablet link 2개에 센서 부착 -> 완료
# Tablet link 2개에 대한 좌표 출력 -> 완료
# Tablet link 2개에 대한 센서 출력 -> 완료