import numpy as np


class PupperConfig:
    def __init__(self):
        # Robot joint limits
        self.MAX_JOINT_TORQUE = 1.0
        self.MAX_LEG_FORCE = 15
        self.REVOLUTE_RANGE = 1.57
        self.PRISMATIC_RANGE = 0.125

        # Robot geometry
        self.LEG_FB = 0.10  # front-back distance from center line to leg axis
        self.LEG_LR = 0.0569  # left-right distance from center line to leg plane
        self.LEG_L = 0.125
        self.ABDUCTION_OFFSET = 0.027  # distance from abduction axis to leg

        self.FOOT_RADIUS = 0.01

        self.HIP_L = 0.001  # 0.06
        self.HIP_W = 0.001  # 0.02
        self.HIP_T = 0.001  # 0.02
        self.HIP_OFFSET = 0.02

        (self.L, self.W, self.T) = (0.276, 0.100, 0.050)

        # Robot inertia params
        self.MASS = 1.00  # kg
        Ix = self.MASS / 12 * (self.W ** 2 + self.T ** 2)
        Iy = self.MASS / 12 * (self.L ** 2 + self.T ** 2)
        Iz = self.MASS / 12 * (self.L ** 2 + self.W ** 2)
        self.INERTIA = np.zeros((3, 3))
        self.INERTIA[0, 0] = Ix
        self.INERTIA[1, 1] = Iy
        self.INERTIA[2, 2] = Iz

        self.FRAME_MASS = 0.800
        self.MODULE_MASS = 0.030
        self.LEG_MASS = 0.030

        self.ARMATURE = 0.0024  # Inertia of rotational joints

        self.JOINT_NOISE = 0.02  # Nm, 1 sigma of gaussian noise
        self.LATENCY = 2  # ms of sense->control latency
        self.UPDATE_PERIOD = 2  # ms between control updates


class EnvironmentConfig:
    def __init__(self):
        self.MU = 1.0  # coeff friction
        self.SIM_STEPS = 10000  # simulation steps to take
        self.DT = 0.001  # timestep [s]


# Software stuff
class QPConfig:
    def __init__(self):
        self.ALPHA = 1e-3  # penalty on the 2norm of foot forces
        self.BETA = (
            1e-1
        )  # penalty coefficient for the difference in foot forces between time steps
        self.GAMMA = (
            200
        )  # scale factor to penalize deviations in angular acceleration compared to linear accelerations
        self.MU = 1.0  # friction coefficient


class SwingControllerConfig:
    def __init__(self):
        self.STEP_HEIGHT = 0.08  # [m]
        self.KP = np.array([400, 400, 2000])  # [Nm/rad, Nm/rad, N/m]


class GaitPlannerConfig:
    def __init__(self):
        self.STEP_LENGTH = 0.2  # [m]
        self.D = 0.6  # [s]


PUPPER_CONFIG = PupperConfig()
QP_CONFIG = QPConfig()
SWING_CONTROLLER_CONFIG = SwingControllerConfig()
GAIT_PLANNER_CONFIG = GaitPlannerConfig()
ENVIRONMENT_CONFIG = EnvironmentConfig()
