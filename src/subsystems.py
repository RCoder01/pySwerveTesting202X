import math
import commands2
import ctre
import navx
import rev
import wpilib
import wpilib.drive
import wpimath.geometry
import wpimath.controller
import wpimath.kinematics

import constants


class SwerveModule(commands2.SubsystemBase):
    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber(f"{self.getName()}/Speed", self.get_state().speed)
        wpilib.SmartDashboard.putNumber(f"{self.getName()}/Angle", self.get_state().angle.degrees())

    def __init__(self, name: str, azimuth_motor_ID: int, power_motor_ID: int, offset: wpimath.geometry.Translation2d) -> None:
        commands2.SubsystemBase.__init__(self)
        self.setName(name)

        self.offset = offset

        self._azimuth_motor = ctre.WPI_TalonSRX(azimuth_motor_ID)
        self._azimuth_motor.configAllSettings(constants.AZIMUTH_CONFIG)

        self._power_motor = rev.CANSparkMax(power_motor_ID, rev.CANSparkMax.MotorType.kBrushless)
        self._power_motor_encoder = self._power_motor.getEncoder()

        self._power_PID = self._power_motor.getPIDController()
        constants.config_power_PID(self._power_PID)

    def get_optimized_azimuth_setpoint(self, setpoint: int):
        HALF_ROTATION = constants.ENCODER_COUNTS_PER_ROTATION_AZIMUTH / 2
        current_position = self._azimuth_motor.getSelectedSensorPosition()
        mod_setpoint = setpoint % constants.ENCODER_COUNTS_PER_ROTATION_AZIMUTH
        error = (current_position % constants.ENCODER_COUNTS_PER_ROTATION_AZIMUTH) - mod_setpoint
        if error <= -HALF_ROTATION:
            return current_position + mod_setpoint - constants.ENCODER_COUNTS_PER_ROTATION_AZIMUTH
        elif -HALF_ROTATION < error < HALF_ROTATION:
            return current_position + mod_setpoint
        elif HALF_ROTATION <= error:
            return current_position + mod_setpoint + constants.ENCODER_COUNTS_PER_ROTATION_AZIMUTH

    def set_azimuth(self, angle: wpimath.geometry.Rotation2d):
        self._azimuth_motor.set(
            ctre.ControlMode.Position,
            self.get_optimized_azimuth_setpoint(
                angle.degrees() * constants.ENCODER_COUNTS_PER_ROTATION_AZIMUTH / 360
            ),
        )

    def set_speed(self, rpm: float):
        self._power_PID.setReference(rpm, rev.CANSparkMax.ControlType.kVelocity)

    def set_state(self, state: wpimath.kinematics.SwerveModuleState):
        if state.speed > constants.CRITICAL_SPEED:
            self.set_azimuth(state.angle)
        self.set_speed(state.speed)

    def get_speed(self) -> float:
        return self._power_motor_encoder.getVelocity()

    def get_azimuth(self):
        return wpimath.geometry.Rotation2d((self._azimuth_motor.getSelectedSensorPosition() * 360 / 4096) or 0)

    def get_state(self):
        return wpimath.kinematics.SwerveModuleState(self.get_speed(), self.get_azimuth())


BigBird = SwerveModule('BigBird', **constants.drivetrain['BigBird'])
BigGiraffe = SwerveModule('BigGiraffe', **constants.drivetrain['BigGiraffe'])
BigHorse = SwerveModule('BigHorse', **constants.drivetrain['BigHorse'])
BigSushi = SwerveModule('BigSushi', **constants.drivetrain['BigSushi'])

class Drivetrain(commands2.SubsystemBase):
    def periodic(self) -> None:
        self._odometry.update(
            self._gyro.getRotation2d(),
            BigBird.get_state(),
            BigGiraffe.get_state(),
            BigHorse.get_state(),
            BigSushi.get_state(),
        )

    def __init__(self) -> None:
        commands2.SubsystemBase.__init__(self)
        self._gyro = navx.AHRS(wpilib.SPI.Port.kMXP)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(BigBird.offset, BigGiraffe.offset, BigHorse.offset, BigSushi.offset)
        self._odometry = wpimath.kinematics.SwerveDrive4Odometry(self.kinematics, self._gyro.getRotation2d())

    def set_chassis_speeds(self, chassis_speeds: wpimath.kinematics.ChassisSpeeds):
        self.set_module_states(*self.kinematics.toSwerveModuleStates(chassis_speeds))

    def set_module_states(
            self,
            bird_state: wpimath.kinematics.SwerveModuleState,
            giraffe_state: wpimath.kinematics.SwerveModuleState,
            horse_state: wpimath.kinematics.SwerveModuleState,
            sushi_state: wpimath.kinematics.SwerveModuleState,
            ):
        tuple(map(
            SwerveModule.set_state,
            (BigBird, BigGiraffe, BigHorse, BigSushi),
            self._optimize_states(bird_state, giraffe_state, horse_state, sushi_state)
        ))

    def _optimize_states(
            self,
            desired_bird_state: wpimath.kinematics.SwerveModuleState,
            desired_giraffe_state: wpimath.kinematics.SwerveModuleState,
            desired_horse_state: wpimath.kinematics.SwerveModuleState,
            desired_sushi_state: wpimath.kinematics.SwerveModuleState,
            ):
        # return tuple(map(
        #     wpimath.kinematics.SwerveModuleState.optimize(desired_bird_state, BigBird.get_azimuth()),
        #     wpimath.kinematics.SwerveModuleState.optimize(desired_giraffe_state, BigGiraffe.get_azimuth()),
        #     wpimath.kinematics.SwerveModuleState.optimize(desired_horse_state, BigHorse.get_azimuth()),
        #     wpimath.kinematics.SwerveModuleState.optimize(desired_sushi_state, BigSushi.get_azimuth()),
        # ))
        return tuple(map(
            wpimath.kinematics.SwerveModuleState.optimize,
            iter((desired_bird_state, desired_giraffe_state, desired_horse_state, desired_sushi_state)),
            map(
                SwerveModule.get_azimuth,
                (BigBird, BigGiraffe, BigHorse, BigSushi),
            ),
        ))

    def get_pose(self):
        return self._odometry.getPose()

    def reset_odometry(self, pose: wpimath.geometry.Pose2d()):
        self._odometry.resetPosition(pose, self._gyro.getRotation2d())

drivetrain = Drivetrain()
