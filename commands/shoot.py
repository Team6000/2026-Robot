import math
import commands2
from wpilib import SmartDashboard
from commands.fancy_driving.aimtodirection_help import AimToDirectionHelper
from constants import ShooterConstants
from wpilib import DriverStation

class AutoShoot(commands2.Command):
    """
    Drives + auto aims + shoots.
    """

    def __init__(self, drivetrain, hopper, shooter):
        super().__init__()

        self.drivetrain = drivetrain
        self.hopper = hopper
        self.shooter = shooter

        # ✅ NOW we REQUIRE drivetrain again
        self.addRequirements(hopper, shooter)

        # Target position
        target = self.getTargetPosition()
        self.target_x = target[0]
        self.target_y = target[1]

       # self.aimHelper = AimToDirectionHelper(self.drivetrain)


    def getTargetPosition(self):
        alliance = DriverStation.getAlliance()

        if alliance == DriverStation.Alliance.kRed:
            return [4.604766, 4.021500]
        else:
            return [4.604766, 4.021500]

    def getDistance(self):
        pose = self.drivetrain.getPose()

        dx = self.target_x - pose.x
        dy = self.target_y - pose.y

        return math.sqrt(dx**2 + dy**2)

    def execute(self):
        distance = self.getDistance()

        SmartDashboard.putNumber("Distance to Hub", distance)
        self.shooter.runCalculatedShooterSpeed(distance)

        if self.shooter.shooter_encoder.getVelocity() > self.shooter.target_rpm - 1000:
            self.shooter.indexer_motor.set(ShooterConstants.shooting_index_velocity)
            #if self.shooter.indexer_encoder.getVelocity() < ShooterConstants.shooting_index_velocity + 1000:
            self.hopper.hopper_motor_spin_inwards()

    def end(self, interrupted):
        self.shooter.stop()
        self.hopper.stop_rolling_motor()

    def isFinished(self):
        return False