import math
import commands2
from wpilib import SmartDashboard
from commands.fancy_driving.aimtodirection_help import AimToDirectionHelper
from constants import ShooterConstants


class AngleShoot(commands2.Command):
    """
    Drives + auto aims + shoots.
    """

    def __init__(self, drivetrain, hopper, shooter, getX, getY, getRot):
        super().__init__()

        self.drivetrain = drivetrain
        self.hopper = hopper
        self.shooter = shooter

        self.getX = getX
        self.getY = getY
        self.getRot = getRot

        # ✅ NOW we REQUIRE drivetrain again
        self.addRequirements(drivetrain, hopper, shooter)

        # Target position
        self.target_x = 11.913234
        self.target_y = 4.021500

        # Aim helper
        self.aimHelper = AimToDirectionHelper(self.drivetrain)

    def getDistance(self):
        pose = self.drivetrain.getPose()

        dx = self.target_x - pose.x
        dy = self.target_y - pose.y

        return math.sqrt(dx**2 + dy**2)

    def getTargetAngle(self):
        pose = self.drivetrain.getPose()

        dx = self.target_x - pose.x
        dy = self.target_y - pose.y

        angle = math.degrees(math.atan2(dy, dx))
        return angle + 180

    def at_Target_Angle(self):
        return self.aimHelper.atTarget(self.getTargetAngle())

    def execute(self):
        distance = self.getDistance()
        target_angle = self.getTargetAngle()

        x = self.getX()
        y = self.getY()
        driver_rot = self.getRot()

        # # Auto aim OR driver override
        # if abs(driver_rot) > 0.1:
        #     rot = driver_rot
        # else:
        #     rot = self.aimHelper.getTurnSpeed(target_angle)
        #
        # # ✅ Drive (FIXED signature)
        # self.drivetrain.drive(x, y, rot, True, True)
        #
        # # ✅ SmartDashboard (UNCHANGED)
        # SmartDashboard.putNumber("Distance to Hub", distance)
        # SmartDashboard.putNumber("Shooter Target Angle", target_angle)
        # SmartDashboard.putBoolean("At Angle", self.at_Target_Angle())
        # SmartDashboard.putNumber(
        #     "Distance Angle to Hub",
        #     abs(self.drivetrain.getPoseHeading().degrees() - target_angle)
        # )

        # Shoot logic
        # if self.at_Target_Angle():
        if True:
            self.shooter.runCalculatedShooterSpeed(distance)

            if self.shooter.shooter_encoder.getVelocity() > self.shooter.target_rpm - 1000:
                self.shooter.indexer_motor.set(ShooterConstants.shooting_index_velocity)
                #if self.shooter.indexer_encoder.getVelocity() < ShooterConstants.shooting_index_velocity + 1000:
                self.hopper.hopper_motor_spin_inwards()

    def end(self, interrupted):
        self.drivetrain.drive(0, 0, 0, True, True)
        self.shooter.stop()
        self.hopper.stop_rolling_motor()

    def isFinished(self):
        return False