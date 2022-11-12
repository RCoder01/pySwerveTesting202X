import math
from ctre import TalonSRXConfiguration
import ctre
import rev
import wpimath.trajectory
import wpimath.geometry

drivetrain = {
    "BigBird": {
        "offset": wpimath.geometry.Translation2d(0.265, -0.2625),
        "azimuth_motor_ID": 0,
        "power_motor_ID": 1,
    },
    "BigGiraffe": {
        "offset": wpimath.geometry.Translation2d(0.265, 0.2625),
        "azimuth_motor_ID": 2,
        "power_motor_ID": 3,
    },
    "BigHorse": {
        "offset": wpimath.geometry.Translation2d(-0.265, 0.2625),
        "azimuth_motor_ID": 4,
        "power_motor_ID": 5,
    },
    "BigSushi": {
        "offset": wpimath.geometry.Translation2d(-0.265, -0.2625),
        "azimuth_motor_ID": 6,
        "power_motor_ID": 7,
    },
}
characterization = {
    "max_speed": 0,
    "max_acceleration": 0,
    "ramsete": {
        "B": 2,
        "Zeta": 0.7,
    }
}
controllers = {
    "power_PID": {"Kp": 0, "Ki": 0, "Kd": 0},
    "power_feedforward": {"kS": 0, "kV": 0, "kA": 0},
    "x_PID": {"Kp": 0, "Ki": 0, "Kd": 0},
    "y_PID": {"Kp": 0, "Ki": 0, "Kd": 0},
    "theta_PID": {"Kp": 0, "Ki": 0, "Kd": 0, "constraints": wpimath.trajectory.TrapezoidProfile.Constraints()},
}


CRITICAL_SPEED = 0.001
ENCODER_COUNTS_PER_ROTATION_AZIMUTH = 4096
WHEEL_DIAMETER = 0.08
ENCODER_COUNTS_PER_ROTATION_POWER = 42
METERS_PER_ENCODER_COUNT_POWER = math.pi * WHEEL_DIAMETER / ENCODER_COUNTS_PER_ROTATION_POWER

AZIMUTH_CONFIG = TalonSRXConfiguration()
AZIMUTH_CONFIG.primaryPID = ctre.BaseTalonPIDSetConfiguration(ctre.FeedbackDevice.CTRE_MagEncoder_Relative)
AZIMUTH_CONFIG.slot0.kP = 0
AZIMUTH_CONFIG.slot0.kI = 0
AZIMUTH_CONFIG.slot0.kD = 0
AZIMUTH_CONFIG.slot0.kF = 0

def config_power_PID(power_PID_controller: rev.SparkMaxPIDController):
    power_PID_controller.setP(0)
    power_PID_controller.setI(0)
    power_PID_controller.setD(0)
    power_PID_controller.setFF(0)
