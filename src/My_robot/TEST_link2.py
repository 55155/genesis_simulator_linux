import genesis as gs

gs.init(backend=gs.cuda, logging_level="debug")

scene = gs.Scene(
    show_viewer=True,
)
plane = scene.add_entity(
    gs.morphs.Plane(),
)
scene.add_entity(
    gs.morphs.Mesh(
        file='/home/seongjin/Desktop/Seongjin/genesis_simulation_on_linux/My_asset/TEST_LINK_sizeup_description/meshes/base_link.stl',
        scale=(0.001, 0.001, 0.001),
    ),
)
scene.build()
# scene.viewer.run()