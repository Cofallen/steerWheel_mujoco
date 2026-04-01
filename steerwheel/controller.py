import time
import math
import mujoco
import mujoco.viewer
import numpy as np
from pynput import keyboard
import threading

dir_path = "scence.xml"
m = mujoco.MjModel.from_xml_path(dir_path)
d = mujoco.MjData(m)

ctrl = np.zeros(m.nu)

# 舵轮和轮子的关节名称
steer_joints = [f"steer{i}" for i in range(4)]
wheel_joints = [f"wheel{i}" for i in range(4)]

# 获取关节id
steer_ids = [m.joint(name).id for name in steer_joints]
wheel_ids = [m.joint(name).id for name in wheel_joints]

# 机器人参数（轮距、轴距、轮半径等）
R = 0.4  # 轴距
r = 0.05 # 轮半径
steer_output = [0.0, 0.0, 0.0, 0.0]  # 舵轮
wheel_output = [0.0, 0.0, 0.0, 0.0]  # 轮子

target = [0, 0, 0]

# 正运动学解算函数
def forward_kinematics(vx, vy, w):
    steer_output[0] = math.atan2(vy + R*w, vx)
    wheel_output[0] = math.sqrt(vx**2 + (vy + R*w)**2) / r
    steer_output[1] = math.atan2(vy, vx - R*w) - math.pi / 2
    wheel_output[1] = math.sqrt((vx - R*w)**2 + vy**2) / r
    steer_output[2] = math.atan2(vy - R*w, vx) + math.pi
    wheel_output[2] = math.sqrt(vx**2 + (vy - R*w)**2) / r
    steer_output[3] = math.atan2(vy, vx + R*w) + math.pi / 2
    wheel_output[3] = math.sqrt((vx + R*w)**2 + vy **2) / r
    
    return steer_output, wheel_output

def on_press(key):
    if key == keyboard.Key.up:
        target[1] = 100.0
    elif key == keyboard.Key.down:
        target[1] = -100.0
    elif key == keyboard.Key.left:
        target[0] = -100.0
    elif key == keyboard.Key.right:
        target[0] = 100.0
    elif key == keyboard.Key.shift_l:
        target[2] = -100.0
    elif key == keyboard.Key.shift_r:
        target[2] = 100.0
    print(1)
    
def on_release(key):
    target[0] = 0
    target[1] = 0
    target[2] = 0

# 键盘监听函数
def listen_keyboard():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

# 启动键盘监听的线程
keyboard_thread = threading.Thread(target=listen_keyboard)
keyboard_thread.daemon = True  # 确保主程序退出时监听器也会退出
keyboard_thread.start()

with mujoco.viewer.launch_passive(m, d) as viewer:
    
    while viewer.is_running():
        step_start = time.time()
        # 读取当前舵轮角度和轮子速度
        steer_angles = [d.qpos[steer_ids[i]] for i in range(4)]
        wheel_speeds = [d.qvel[wheel_ids[i]] for i in range(4)]
        # 正运动学解算
        forward_kinematics(target[0], target[1], target[2])
    
        # print(f"Steer angles: {steer_output}, Wheel speeds: {wheel_output}")

        # 控制器：将目标写入控制量
        for i in range(4):
            d.ctrl[i] = steer_output[i]  # 前4个是舵轮位置控制
            d.ctrl[4 + i] = wheel_output[i]  # 后4个是轮子速度控制
        mujoco.mj_step(m, d)
        viewer.sync()
        # 控制仿真步长
        time.sleep(max(0, m.opt.timestep - (time.time() - step_start)))