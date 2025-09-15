import os

# OBJ 파일이 있는 폴더 경로 (실제 경로로 수정)
folder_path = "/home/seongjin/Desktop/Seongjin/genesis_simulation_on_linux/My_asset/Crank_slider_system_V3_description/urdf/"

# 폴더 내 모든 .obj 파일 리스트
obj_files = [f for f in os.listdir(folder_path) if f.lower().endswith('.obj')]

# scale 파라미터 (필요시 수정)
scale_str = '0.001 0.001 0.001'

# 출력할 문자열 리스트
output_lines = []
for obj_file in sorted(obj_files):
    abs_path = os.path.join(folder_path, obj_file)
    Temp = os.path.splitext(obj_file)[0]
    output_lines.append(f'<mesh name="{Temp}" file="{abs_path}" scale="{scale_str}"/>')

# txt 파일로 저장
output_path = os.path.join(folder_path, 'mesh_list.txt')
with open(output_path, 'w') as f:
    f.write('\n'.join(output_lines))
