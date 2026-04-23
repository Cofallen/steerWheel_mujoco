import mujoco
import numpy as np

class RobotWriter:
    def __init__(self, model, data):
        self.m = model
        self.d = data

        steer_joints = [f"steer{i}" for i in range(4)]
        wheel_joints = [f"wheel{i}" for i in range(4)]

        self.steer_ids = [self.m.joint(name).id for name in steer_joints]
        self.wheel_ids = [self.m.joint(name).id for name in wheel_joints]
        
        steer_actuators = [f"steer{i}" for i in range(4)]
        wheel_actuators = [f"wheel{i}" for i in range(4)]

        self.steer_act_ids = [self.m.actuator(name).id for name in steer_actuators]
        self.wheel_act_ids = [self.m.actuator(name).id for name in wheel_actuators]

    def write(self, steer_output, wheel_output):
        for i in range(4):
            self.d.ctrl[self.steer_act_ids[i]] = steer_output[i]
            self.d.ctrl[self.wheel_act_ids[i]] = wheel_output[i]

    def read(self):
        steer_angles = [self.d.qpos[self.steer_ids[i]] for i in range(4)]
        wheel_angles = [self.d.qpos[self.wheel_ids[i]] for i in range(4)]
        return steer_angles, wheel_angles