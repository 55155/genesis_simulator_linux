'''
import mujoco
import yaml
import xml.etree.ElementTree as ET
import re

def replace_package_paths_in_xacro(xacro_content: str, robot_name: str) -> str:
    pattern = re.compile(rf'f\"package://{{{robot_name}_description}}\"')
    replaced_content = pattern.sub('\"..\"', xacro_content)
    return replaced_content

def fix_inertia_values(urdf_path, output_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # 모든 <inertial> 태그 안의 <inertia> 검사
    for inertial in root.findall('.//inertial'):
        inertia = inertial.find('inertia')
        if inertia is not None:
            ixx = float(inertia.get('ixx', '0'))
            iyy = float(inertia.get('iyy', '0'))
            izz = float(inertia.get('izz', '0'))

            # 0 이하 값은 MIN_INERTIA_VALUE로 변경
            if ixx <= 0:
                inertia.set('ixx', str(MIN_INERTIA_VALUE))
            if iyy <= 0:
                inertia.set('iyy', str(MIN_INERTIA_VALUE))
            if izz <= 0:
                inertia.set('izz', str(MIN_INERTIA_VALUE))
    # 수정된 URDF 저장
    tree.write(output_path, encoding='utf-8', xml_declaration=True)



if __name__ == "__main__":
    with open("/home/seongjin/Desktop/Seongjin/genesis_simulation_on_linux/src/config.yaml", 'r') as file:
        config = yaml.safe_load(file)

    asset_path = config["file_path"]["asset"]
    Robot_name = "Crank_slider_system_V1"
    file_path = asset_path + Robot_name + "_description" + "/urdf/"

    with open(file_path + Robot_name + ".urdf.xacro", 'r') as file:
        xacro_content = file.read()
    new_xacro_content = replace_package_paths_in_xacro(xacro_content, Robot_name)
    
    with open(file_path + Robot_name + ".urdf", 'w') as file:
        file.write(new_xacro_content)

    model = mujoco.MjModel.from_xml_path(file_path + Robot_name + ".urdf")
    mujoco.mj_saveLastXML(file_path+Robot_name+".xml", model)


    MIN_INERTIA_VALUE = 1e-6  # 최소 관성값 (0보다 작은 값 대체용)

    # 함수 실행 예시
    fix_inertia_values(file_path+Robot_name, output_urdf_path)
    print(f"Modified URDF saved to {output_urdf_path}")

'''
import mujoco
import yaml
import xml.etree.ElementTree as ET
import re
import os

MIN_INERTIA_VALUE = 1e-6  # 최소 관성값

def replace_package_paths_in_xacro(xacro_content: str, robot_name: str) -> str:
    # package:// 경로 치환
    pattern1 = re.compile(rf'package://{robot_name}_description')
    replaced_content = pattern1.sub('..', xacro_content)
    
    # $(find ...) 형태 치환 (예: $(find Crank_slider_system_V1_description))
    pattern2 = re.compile(rf'\$\(\s*find\s+{robot_name}_description\s*\)')
    replaced_content = pattern2.sub('..', replaced_content)
    
    return replaced_content


def fix_inertia_values(urdf_path, output_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for inertial in root.findall('.//inertial'):
        inertia = inertial.find('inertia')
        if inertia is not None:
            ixx = float(inertia.get('ixx', '0'))
            iyy = float(inertia.get('iyy', '0'))
            izz = float(inertia.get('izz', '0'))

            # MIN_INERTIA_VALUE 이하일 경우 수정
            if ixx <= MIN_INERTIA_VALUE:
                inertia.set('ixx', str(MIN_INERTIA_VALUE))
            if iyy <= MIN_INERTIA_VALUE:
                inertia.set('iyy', str(MIN_INERTIA_VALUE))
            if izz <= MIN_INERTIA_VALUE:
                inertia.set('izz', str(MIN_INERTIA_VALUE))

    tree.write(output_path, encoding='utf-8', xml_declaration=True)

if __name__ == "__main__":
    with open("/home/seongjin/Desktop/Seongjin/genesis_simulation_on_linux/src/config.yaml", 'r') as file:
        config = yaml.safe_load(file)

    asset_path = config["file_path"]["asset"]
    Robot_name = "Crank_slider_system_V1"
    file_path = asset_path + Robot_name + "_description" + "/urdf/"
    xacro_file = file_path + Robot_name + ".xacro"
    urdf_file = file_path + Robot_name + ".urdf"
    output_urdf_path = file_path + Robot_name + "_fixed.urdf"
    
    if not os.path.exists(output_urdf_path):

        # xacro 내에서 package 경로 치환
        with open(xacro_file, 'r') as file:
            xacro_content = file.read()
        new_xacro_content = replace_package_paths_in_xacro(xacro_content, Robot_name)
        with open(xacro_file, 'w') as file:
            file.write(new_xacro_content)

        # xacro -> urdf 변환
        os.system(f"xacro {xacro_file} -o {urdf_file}")

        # urdf 파일 경로 package:// 수정
        with open(urdf_file, 'r') as file:
            urdf_content = file.read()
        new_urdf_content = replace_package_paths_in_xacro(urdf_content, Robot_name)
        with open(urdf_file, 'w') as file:
            file.write(new_urdf_content)

        # 관성값 수정
        fix_inertia_values(urdf_file, output_urdf_path)
        print(f"Modified URDF saved to {output_urdf_path}")

    # urdf -> mjcf 변환

    # mjcf stl 위치 수정
    os.system(f"cd {file_path} && cp ../col_meshes/* {file_path}")

    model = mujoco.MjModel.from_xml_path(output_urdf_path)
    mujoco.mj_saveLastXML(file_path + Robot_name + ".xml", model)

