import mujoco

with open('/home/seongjin/Desktop/Seongjin/genesis_simulation_on_linux/My_asset/Crank_slider_system_V3_Pjoint_description/urdf/Crank_slider_system_V3_Pjoint.xml', 'r') as f:
    XML = f.read()
ASSETS=dict()

import os
stl_file_list = [stl for stl in os.listdir('/home/seongjin/Desktop/Seongjin/genesis_simulation_on_linux/My_asset/Crank_slider_system_V3_Pjoint_description/urdf/') if stl.endswith('.stl')] 
print(stl_file_list)
for stl_file in stl_file_list:
   with open('/home/seongjin/Desktop/Seongjin/genesis_simulation_on_linux/My_asset/Crank_slider_system_V3_Pjoint_description/urdf/'+stl_file, 'rb') as f:
       ASSETS[stl_file] = f.read()


model = mujoco.MjModel.from_xml_string(XML, ASSETS)
data = mujoco.MjData(model)
print(data.geom_xpos[0])
print(data.)

# while data.time < 1:
#   mujoco.mj_step(model, data)
#   print(data.geom_xpos)