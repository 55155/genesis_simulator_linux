import genesis as gs
import numpy as np
import yaml 

with open('./src/config.yaml', 'r') as file:
    config = yaml.load(file, Loader=yaml.Loader)

gs.init(backend=gs.metal)
scene = gs.Scene(
    show_viewer=True,
    sim_options= gs.options.SimOptions(
        dt = 0.0001,
        gravity=(0.0, 0.0, -9.81),
    ),
    viewer_options=gs.options.ViewerOptions(
        run_in_thread=False,
        res=(1280, 960),
        camera_pos=(1, 0.0, 0.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=40,
        max_FPS=60,
    ),
    vis_options=gs.options.VisOptions(
        show_world_frame=True,
        world_frame_size=1.0,
        show_link_frame=False,
        show_cameras=False,
        plane_reflection=True,
        ambient_light=(0.1, 0.1, 0.1),
    ),
    # renderer=gs.renderers.RayTracer(),
    renderer=gs.renderers.Rasterizer(),
)

plane = scene.add_entity(gs.morphs.Plane(
    pos = (0, 0, 0),
))

# adding Crank_slider_system_copy mjcf entity to the scene

fn = './My_asset/Crank_slider_system_copy_description/urdf/Crank_slider_system_copy_modified_inertia.xml'
my_link = scene.add_entity(
    gs.morphs.MJCF(file = fn, euler = (90,0,0), pos = (-0.3, 0.0, 0), scale = 1.0, decimate = False, convexify = False,),
)
scene.build()   

## scene에 모든 엔티티 추가 후에 buildS
iter = 1000
for i in range(iter):
    scene.step()

