import commands2
import wpilib

import subsystems
import commands


class Robot(commands2.TimedCommandRobot):
    def __init__(self) -> None:
        commands2.TimedCommandRobot.__init__(self)

    def teleopInit(self) -> None:
        if getattr(self, '_auton_command', None) is not None:
            self._auton_command.cancel()
        subsystems.drivetrain.setDefaultCommand(commands.TeleopDrive())

    def autonomousInit(self) -> None:
        self._auton_command: commands2.CommandBase = None

if __name__ == '__main__':
    wpilib.run(Robot)
