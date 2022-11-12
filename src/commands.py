import math
import typing
import commands2
import wpilib
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import wpimath.trajectory.constraint
import wpimath.kinematics
import wpimath.spline

import constants
import subsystems


class TeleopDrive(commands2.CommandBase):
    def __init__(self) -> None:
        commands2.CommandBase.__init__(self)
        self.addRequirements(subsystems.drivetrain)
        self.setName('Drivetrain teleop drive')

        driver = wpilib.XboxController(0)
        self._vx_supplier = driver.getLeftX
        self._vy_supplier = driver.getLeftY
        self._omega_supplier = driver.getRightX

    def initialize(self) -> None:
        subsystems.drivetrain.set_chassis_speeds(wpimath.kinematics.ChassisSpeeds(0, 0, subsystems.drivetrain.get_pose().rotation().radians()))

    def execute(self) -> None:
        subsystems.drivetrain.set_chassis_speeds(
            wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                self._vx_supplier(), self._vy_supplier(), self._omega_supplier(), subsystems.drivetrain.get_pose().rotation(),
            )
        )

    def isFinished(self) -> bool:
        return False

    def end(self):
        subsystems.drivetrain.set_chassis_speeds(wpimath.kinematics.ChassisSpeeds(0, 0, subsystems.drivetrain.get_pose().rotation()))


class DriveTrajectory(commands2.Swerve4ControllerCommand):
    @typing.overload
    def __init__(self, controlVectors: typing.List[wpimath.spline.Spline5.ControlVector], /) -> None:
        pass
    @typing.overload
    def __init__(self, initial: wpimath.spline.Spline3.ControlVector, interiorWaypoints: typing.List[wpimath.geometry.Translation2d], end: wpimath.spline.Spline3.ControlVector, /) -> None:
        pass
    @typing.overload
    def __init__(self, start: wpimath.geometry.Pose2d, interiorWaypoints: typing.List[wpimath.geometry.Translation2d], end: wpimath.geometry.Pose2d, /) -> None:
        pass
    @typing.overload
    def __init__(self, waypoints: typing.List[wpimath.geometry._geometry.Pose2d], /) -> None:
        pass
    def __init__(self, *args, desired_rotations: typing.Callable[[], wpimath.geometry._geometry.Rotation2d] = None):
        trajectory_config = wpimath.trajectory.TrajectoryConfig(
            constants.characterization["max_speed"],
            constants.characterization["max_acceleration"],
        )
        trajectory_config.setKinematics(subsystems.drivetrain.kinematics)
        trajectory_config.addConstraint(
            wpimath.trajectory.constraint.SwerveDrive4KinematicsConstraint(
                subsystems.drivetrain.kinematics,
                constants.characterization["max_speed"],
            )
        )
        kwargs = {"desiredRotation": desired_rotations} if desired_rotations is not None else {}
        theta_controller = wpimath.controller.ProfiledPIDControllerRadians(**constants.controllers['theta_PID'])
        theta_controller.enableContinuousInput(0, 360)
        commands2.Swerve4ControllerCommand.__init__(
            self,
            wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
                *args,
                config=trajectory_config,
            ),
            subsystems.drivetrain.get_pose,
            subsystems.drivetrain.kinematics,
            wpimath.controller.PIDController(**constants.controllers['x_PID']),
            wpimath.controller.PIDController(**constants.controllers['y_PID']),
            theta_controller,
            subsystems.drivetrain.set_module_states,
            **kwargs,
        )

    def end(self, interrupted: bool):
        subsystems.drivetrain.set_chassis_speeds(wpimath.kinematics.ChassisSpeeds(0, 0, subsystems.drivetrain.get_pose().rotation()))
        super().end()
