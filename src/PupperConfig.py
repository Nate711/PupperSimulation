import numpy as np


class PupperConfig:
    def __init__(self):
        # XML files
        self.XML_IN = "pupper.xml"
        self.XML_OUT = "pupper_out.xml"

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

        self.HIP_L = 0.06
        self.HIP_W = 0.02
        self.HIP_T = 0.02
        self.HIP_OFFSET = 0.02

        (self.L, self.W, self.T) = (0.276, 0.100, 0.050)

        # Robot inertia params
        self.FRAME_MASS = 0.800  # kg
        self.MODULE_MASS = 0.030  # kg
        self.LEG_MASS = 0.030  # kg
        self.MASS = self.FRAME_MASS + (self.MODULE_MASS + self.LEG_MASS)*4
        Ix = self.MASS / 12 * (self.W ** 2 + self.T ** 2)
        Iy = self.MASS / 12 * (self.L ** 2 + self.T ** 2)
        Iz = self.MASS / 12 * (self.L ** 2 + self.W ** 2)
        self.INERTIA = np.zeros((3, 3))
        self.INERTIA[0, 0] = Ix
        self.INERTIA[1, 1] = Iy
        self.INERTIA[2, 2] = Iz

        self.FRAME_INERTIA = "0.0065733 0.074011 0.077763"
        self.MODULE_INERTIA = "0.002449 0.005043 0.006616 -0.001784 -.00002 -0.000007"
        self.LEG_INERTIA = "0.003575 0.006356 0.002973 -0.0001326 -0.0001079 -0.0002538"



        # Joint params
        self.ARMATURE = 0.0024  # Inertia of rotational joints

        NATURAL_DAMPING = 0.01  # Damping resulting from friction
        ELECTRICAL_DAMPING = 0.049  # Damping resulting from back-EMF
        self.REV_DAMPING = (
            NATURAL_DAMPING + ELECTRICAL_DAMPING
        )  # Torque damping on the revolute joints
        self.PRISM_DAMPING = 0.2  # Frictional damping on the prismatic joints

        # Servo params
        self.SERVO_REV_KP = 100  # Position gain [Nm/rad]
        self.SERVO_PRISM_KP = 10000  # Position gain [N/m]

        # Noise
        self.JOINT_NOISE = 0.02  # Nm, 1 sigma of gaussian noise
        self.LATENCY = 2  # ms of sense->control latency
        self.UPDATE_PERIOD = 2  # ms between control updates


class EnvironmentConfig:
    def __init__(self):
        self.MU = 1.0  # coeff friction
        self.SIM_STEPS = 10000  # simulation steps to take
        self.DT = 0.001  # timestep [s]


class GaitPlannerConfig:
    def __init__(self):
        self.STEP_LENGTH = 0.2  # [m]
        self.D = 0.6  # [s]


class SolverConfig:
    def __init__(self):
        self.JOINT_SOLREF = "0.001 1"  # time constant and damping ratio for joints
        self.JOINT_SOLIMP = "0.9 0.95 0.001"  # joint constraint parameters
        self.GEOM_SOLREF = (
            "0.005 2"
        )  # time constant and damping ratio for geom contacts
        self.GEOM_SOLIMP = "0.9 0.95 0.001"  # geometry contact parameters


PUPPER_CONFIG = PupperConfig()
GAIT_PLANNER_CONFIG = GaitPlannerConfig()
ENVIRONMENT_CONFIG = EnvironmentConfig()
SOLVER_CONFIG = SolverConfig()
