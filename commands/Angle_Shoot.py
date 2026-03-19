
import math
import commands2
from wpilib import SmartDashboard
from commands.fancy_driving.manual_aimtodirection import AimToDirection

class AngleShoot(commands2.Command):
    """
    Uses Limelight pose estimation to determine the correct
    robot angle and shooter angle for scoring.
    """

    def __init__(self, drivetrain, hopper, shooter):
        super().__init__()

        self.drivetrain = drivetrain
        self.hopper = hopper
        self.shooter = shooter
        self.addRequirements(drivetrain, hopper, shooter) #this cancels the other commands and makes sure that it does this

        team = "Red"
        # Function to get team:
        if team == "Red":
            self.target_x = 11.913234
            self.target_y = 4.021500
        else:
            self.target_x = 4.604766
            self.target_y = 4.021500

        self.distance = 0
        self.shooter_angle = 0
        self.aimCommand = None


    def getDistance(self):
        """Calculate distance from robot to target"""

        pose = self.drivetrain.getPose()

        robot_x = pose.x
        robot_y = pose.y

        dx = self.target_x - robot_x
        dy = self.target_y - robot_y

        self.distance = math.sqrt(dx**2 + dy**2)

        return self.distance

    def getTargetAngle(self):
        """Calculate angle robot should face"""

        pose = self.drivetrain.getPose()

        robot_x = pose.x
        robot_y = pose.y

        dx = self.target_x - robot_x
        dy = self.target_y - robot_y

        angle = math.degrees(math.atan2(dy, dx))

        reversed_angle = angle + 180 # Back of Robot

        return reversed_angle

    def initialize(self):

        # Create AimToDirection command using calculated angle
        self.aimCommand = AimToDirection(
            self.getTargetAngle(),
            self.drivetrain
        )

        self.aimCommand.initialize()

    def at_Target_Angle(self):
        return abs(self.drivetrain.getPoseHeading().degrees() - self.getTargetAngle()) < 5

    def execute(self):
        SmartDashboard.putNumber("Distance to Hub", self.getDistance())
        SmartDashboard.putNumber("Shooter Target Angle", self.getTargetAngle())
        SmartDashboard.putBoolean("At Angle", self.at_Target_Angle())
        SmartDashboard.putNumber("Distance Angle to Hub", abs(self.drivetrain.getPoseHeading().degrees() - self.getTargetAngle()))
        self.aimCommand.execute()
        # TODO: ONLY RUN HOPPER WHEN SHOOTER AT SPEED

        if self.at_Target_Angle():
            self.shooter.runCalculatedShooterSpeed(self.getDistance())
            self.hopper.hopper_motor_spin_inwards()


    def end(self, interrupted):
        self.aimCommand.end(interrupted)
        self.shooter.stop()
        self.hopper.stop_rolling_motor()