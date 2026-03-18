
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
            self.target_x = 16.54
            self.target_y = 5.5
        else:
            self.target_x = 0
            self.target_y = 5.5

        self.distance = 0
        self.shooter_angle = 0
        self.aimCommand = None

    def periodic(self):
        SmartDashboard.putNumber("Distance to Hub", self.getDistance())
        SmartDashboard.putNumber("Shooter Angle", self.getTargetAngle())


    def getDistance(self):
        """Calculate distance from robot to target"""

        pose = self.drivetrain.getPose()

        robot_x = pose[0]
        robot_y = pose[1]

        dx = self.target_x - robot_x
        dy = self.target_y - robot_y

        self.distance = math.sqrt(dx**2 + dy**2)

        return self.distance

    def getTargetAngle(self):
        """Calculate angle robot should face"""

        pose = self.drivetrain.getPose()

        robot_x = pose[0]
        robot_y = pose[1]

        dx = self.target_x - robot_x
        dy = self.target_y - robot_y

        angle = math.degrees(math.atan2(dy, dx))

        return angle

    def initialize(self):

        # Create AimToDirection command using calculated angle
        self.aimCommand = AimToDirection(
            self.getTargetAngle(),
            self.drivetrain
        )

        self.aimCommand.initialize()

    def at_Target_Angle(self):
        return self.drivetrain.getPoseHeading() - self.getTargetAngle() > .5

    def execute(self):
        self.aimCommand.execute()

        if self.at_Target_Angle():
            self.shooter.runCalculatedShooterSpeed(self.getDistance())


    def end(self, interrupted):
        self.aimCommand.end(interrupted)
        self.shooter.stop()

    def isFinished(self):
        return self.aimCommand.isFinished()