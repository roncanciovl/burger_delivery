import os
import re
import sys

# Script para aplicar las mejoras de suavidad de movimiento de Kinova
# al driver y descripciones oficiales instalados en este workspace.
# Este script:
# 1. Reduce el ciclo de latencia interno de hardware (UDP Router Timeout a 200ms) en ros2_kortex.
# 2. Obliga el uso de comunicación del bus interno para el Gripper, unificando movimientos en URDF.
# 3. Limpia configuraciones de simulación defectuosas que causan estoteros y retrasos.

def patch_hardware_interface(workspace_dir):
    filepath = os.path.join(workspace_dir, "ros2_kortex", "kortex_driver", "src", "hardware_interface.cpp")
    if not os.path.exists(filepath):
        print(f"[!] Archivo no encontrado: {filepath}. ¿ros2_kortex está instalado en el workspace?")
        return False
        
    with open(filepath, 'r') as f:
        content = f.read()

    # Reducimos los retrasos del timeout a 200 ms explícitos.
    content = content.replace('router_udp_realtime_.SetMessageTimeout(500);', '')
    content = content.replace('router_udp_realtime_.SetMessageTimeout(1000);', '')
    
    options = 'k_api::RouterClientSendOptions{false, 0, 200}'
    content = re.sub(r'k_api::RouterClientSendOptions\{false, 0, \d+\}', options, content)

    with open(filepath, 'w') as f:
        f.write(content)
    print("[x] Parcheado: hardware_interface.cpp (UDP Latency Timeout reducido a 200ms -> Movimiento fluido)")
    return True

def patch_xacro_descriptions(workspace_dir):
    kortex_description_path = os.path.join(workspace_dir, "ros2_kortex", "kortex_description")
    
    files_to_patch = [
        "grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro",
        "grippers/robotiq_2f_140/urdf/robotiq_2f_140_macro.xacro",
        "grippers/gen3_lite_2f/urdf/gen3_lite_2f.ros2_control.xacro",
        "robots/kortex_robot.xacro",
        "robots/gen3.xacro",
        "arms/gen3/6dof/urdf/gen3_macro.xacro",
        "arms/gen3/6dof/urdf/kortex.ros2_control.xacro",
        "arms/gen3/7dof/urdf/kortex.ros2_control.xacro",
    ]
    
    patterns = [
        r'\bsim_gazebo\s*[:=]\s*"\${[^}]*}"', r'\bsim_gazebo\s*[:=]\s*[^ \t\n>]+',
        r'\bsim_isaac\s*[:=]\s*"\${[^}]*}"', r'\bsim_isaac\s*[:=]\s*[^ \t\n>]+',
        r'\bmock_sensor_commands\s*[:=]\s*"\${[^}]*}"', r'\bmock_sensor_commands\s*[:=]\s*[^ \t\n>]+',
        r'\bisaac_joint_commands\s*[:=]\s*"\${[^}]*}"', r'\bisaac_joint_commands\s*[:=]\s*[^ \t\n>]+',
        r'\bisaac_joint_states\s*[:=]\s*"\${[^}]*}"', r'\bisaac_joint_states\s*[:=]\s*[^ \t\n>]+',
        r'<param\s+name="(?:sim_gazebo|sim_isaac|mock_sensor_commands|isaac_joint_commands|isaac_joint_states)">[^<]*</param>',
        r'\bsim_gazebo\s*(?::=\s*[^ \t\n,"]+)?', r'\bsim_isaac\s*(?::=\s*[^ \t\n,"]+)?',
        r'\bmock_sensor_commands\s*(?::=\s*[^ \t\n,"]+)?', r'\bisaac_joint_commands\s*(?::=\s*[^ \t\n,"]+)?',
        r'\bisaac_joint_states\s*(?::=\s*[^ \t\n,"]+)?',
    ]

    for rel_file in files_to_patch:
        file_path = os.path.join(kortex_description_path, rel_file)
        if not os.path.exists(file_path):
            continue
        
        with open(file_path, "r") as f:
            content = f.read()
        
        new_content = content
        new_content = re.sub(r'\s+or\s+sim_gazebo', '', new_content)
        new_content = re.sub(r'sim_gazebo\s+or\s+', '', new_content)
        new_content = re.sub(r'\s+or\s+sim_isaac', '', new_content)
        new_content = re.sub(r'sim_isaac\s+or\s+', '', new_content)

        for pattern in patterns:
            new_content = re.sub(pattern, '', new_content)
        
        # Activar el Bus Interno del Gripper si es XACRO principal de la base
        if "gen3.xacro" in rel_file or "kortex_robot.xacro" in rel_file:
            new_content = re.sub(r'<xacro:arg\s+name="use_internal_bus_gripper_comm"\s+default="false"\s*/>', 
                                 '<xacro:arg name="use_internal_bus_gripper_comm" default="true" />', new_content)

        if new_content != content:
            with open(file_path, "w") as f:
                f.write(new_content)
            print(f"[x] Parcheado XACRO (Limpieza de parámetros restrictivos / Bus Gripper On): {rel_file}")

def main():
    # Obtiene src, asumiendo que el script corre desde src/burger_delivery/scripts
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_src_dir = os.path.abspath(os.path.join(current_dir, "..", ".."))

    print("--- INICIANDO PARCHEO DE MEJORAS DE RENDIMIENTO DE MOVIMIENTO KINOVA ---")
    patch_hardware_interface(workspace_src_dir)
    print("--- PARCHEANDO DESCRIPCIONES XACRO ---")
    patch_xacro_descriptions(workspace_src_dir)
    
    print("\n[OK] Parches aplicados con exito.")
    print("Por favor recompilar el workspace: colcon build --symlink-install --packages-select kortex_driver kortex_description")

if __name__ == "__main__":
    main()
