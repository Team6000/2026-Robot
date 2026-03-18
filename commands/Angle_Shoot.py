
import math
import commands2
from subsystems import shooter
from commands.fancy_driving.manual_aimtodirection import AimToDirection
#TODO: Make a command with a button tied to it

class AngleShoot(commands2.Command):
    """
    Uses Limelight pose estimation to determine the correct
    robot angle and shooter angle for scoring.
    """

    def __init__(self, drivetrain):
        super().__init__()

        self.drivetrain = drivetrain
        self.shooter = shooter.Shooter()
        self.addRequirements(drivetrain) #this cancels the other commands and makes sure that it does this

        # Speaker position on field (CHANGE if needed)
        self.target_x = 16.54 #For blue, x=0
        self.target_y = 5.55

        self.distance = 0
        self.shooter_angle = 0
        self.aimCommand = None

    # LIMELIGHT POSE FUNCTIONS

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

    def At_Target_Angle(self):
        return self.drivetrain.getPoseHeading() - self.getTargetAngle() > .5

    def execute(self):
        self.aimCommand.execute()

        if self.At_Target_Angle():
            self.shooter.runCalculatedShooterSpeed(self.getDistance())


    def end(self, interrupted):
        self.aimCommand.end(interrupted)

    def isFinished(self):
        return self.aimCommand.isFinished()