from __future__ import annotations

import commands2
import typing

from wpimath.geometry import Pose2d, Rotation2d, Translation3d
from commands2 import RunCommand, CommandScheduler, InstantCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, SmartDashboard

from constants import OIConstants
from subsystems import hopper
from subsystems.drivesubsystem import DriveSubsystem

from commands.reset_xy import ResetXY, ResetSwerveFront
from pathplannerlib.auto import AutoBuilder
from commands.fancy_driving.manual_aimtodirection import AimToDirection
from commands.fancy_driving.pathplanner_to_pose import PathToPose
from commands.fancy_driving.pathplanner_to_path import PathToPath
from commands.swervedrive import SwerveDrive
from commands.Angle_Shoot import AngleShoot
from commands.intake_command import IntakeCommand

from subsystems.limelight_camera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer
from subsystems.shooter import Shooter


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        self.hopper = hopper.Hopper()
        self.shooter = Shooter()
        # Limelight:
        self.limelightLocalizer = LimelightLocalizer(self.robotDrive)

        self.Limelight = LimelightCamera("limelight")

        self.limelightLocalizer.addCamera(
           self.Limelight,
           cameraPoseOnRobot=Translation3d(x=0.1778, y=-0.2921, z=0.43),
           cameraHeadingOnRobot=Rotation2d.fromDegrees(0.0),
           cameraPitchAngleDegrees=0
        )


        # Creates commands
        #self.PathToPose = PathToPose(self.robotDrive, Pose2d(5, 6, Rotation2d(0))) # Drives to the Pose (5,6)
        #self.PathToPath = PathToPath("Path1", self.robotDrive) # Drives to Path1 then follows it
        self.IntakeCommand = IntakeCommand(self.shooter, self.hopper)

        # self.Angle_Shoot.initialize()
        # self.IntakeCommand.initialize()


        # Creates the driver's controller
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.subsystemController = CommandGenericHID(OIConstants.kSubsystemControllerPort)

        # Configure the button bindings and auto chooser
        self.configureButtonBindings()
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Puts the auto chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # Sets the default command of the DriveTrain to be just normal driving
        self.robotDrive.setDefaultCommand(
            SwerveDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=True,
                square=True,
            )
        )


    def configureButtonBindings(self) -> None:
        """
        This function is used to set all the buttons to what they run
        """

        # When x is clicked it resets the pose of the robot
        xButton = self.driverController.button(XboxController.Button.kX)
        xButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))

        # When y is clicked it resets the front of the robot
        yButton = self.driverController.button(XboxController.Button.kY)
        yButton.onTrue(ResetSwerveFront(self.robotDrive))

        # When you press the right bumper set the wheels to x formation
        rbButton = self.driverController.button(XboxController.Button.kRightBumper)
        rbButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))

        # # When you hold a: run the PathToPose or PathToPath command depending on what I have
        aButton = self.driverController.button(XboxController.Button.kA)
        # aButton.onTrue(self.PathToPose)
        # When you let go cancel the command
        aButton.onFalse(RunCommand(lambda: CommandScheduler.getInstance().cancelAll()))

        # # while I hold b aim to the given direction-
        # bButton = self.driverController.button(XboxController.Button.kB)
        # aim_to_direction = AimToDirection(50.0, self.robotDrive)
        # bButton.whileTrue(aim_to_direction)

        # When left bumper is clicked drive forward (robot relative) at 0.1 speeds
        lbButton = self.driverController.button(XboxController.Button.kLeftBumper)
        lbButton.whileTrue(RunCommand(lambda: self.robotDrive.ArcadeDrive(0.1,0), self.robotDrive))


        # Subsystem Controller
        aSubButton = self.subsystemController.button(XboxController.Button.kA)
        aSubButton.whileTrue(RunCommand(lambda: self.hopper.hopper_motor_spin_outwards(), self.hopper))
        aSubButton.onFalse(InstantCommand(lambda: self.hopper.stop_rolling_motor(), self.hopper))

        bSubButton = self.subsystemController.button(XboxController.Button.kB)
        bSubButton.whileTrue(RunCommand(lambda: self.hopper.hopper_motor_spin_inwards(), self.hopper))
        bSubButton.onFalse(InstantCommand(lambda: self.hopper.stop_rolling_motor(), self.hopper))


        xSubButton = self.subsystemController.button(XboxController.Button.kX)
        xSubButton.whileTrue(RunCommand(lambda: self.hopper.extend_hopper(), self.hopper))
        xSubButton.onFalse(InstantCommand(lambda: self.hopper.stop_linear_motor(), self.hopper))

        ySubButton = self.subsystemController.button(XboxController.Button.kY)
        ySubButton.whileTrue(RunCommand(lambda: self.hopper.retract_hopper(), self.hopper))
        ySubButton.onFalse(InstantCommand(lambda: self.hopper.stop_linear_motor(), self.hopper))


        # Intaking
        rbSubButton = self.subsystemController.button(XboxController.Button.kRightBumper)
        rbSubButton.whileTrue(self.IntakeCommand)

        # Shooting
        self.Angle_Shoot = AngleShoot(
            self.robotDrive,
            self.hopper,
            self.shooter,

            getX=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
            getY=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
            getRot=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),
        )
        lbSubButton = self.subsystemController.button(XboxController.Button.kLeftBumper)
        lbSubButton.whileTrue(self.Angle_Shoot)
        #lbSubButton.whileTrue(RunCommand(lambda: self.shooter.runCalculatedShooterSpeed(10), self.shooter))
        #lbSubButton.onFalse(self.Angle_Shoot.end(True))



        # aButton.whileTrue(RunCommand(lambda: self.shooter.runCalculatedShooterSpeed(5, forward=True), self.shooter))
        # bButton.whileTrue(RunCommand(lambda: self.shooter.runCalculatedShooterSpeed(5, forward=False), self.shooter))


    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""


    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        # """
        return self.autoChooser.getSelected()



    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
