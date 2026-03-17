
import math
import commands2

from commands.fancy_driving.manual_aimtodirection import AimToDirection


class AngleShoot(commands2.Command):
    """
    Uses Limelight pose estimation to determine the correct
    robot angle and shooter angle for scoring.
    """

    def __init__(self, drivetrain, shooter):
        super().__init__()

        self.drivetrain = drivetrain
        self.shooter = shooter

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

    # -----------------------------
    # SHOOTER ANGLE CALCULATION
    # -----------------------------

    def calculateShooterAngle(self):
        """
        Converts distance to shooter angle.
        Adjust these numbers based on testing.
        """

        distance = self.getDistance()

        if distance < 2:
            self.shooter_angle = 30
        elif distance < 4:
            self.shooter_angle = 40
        elif distance < 6:
            self.shooter_angle = 50
        else:
            self.shooter_angle = 55

        return self.shooter_angle

    # -----------------------------
    # COMMAND FUNCTIONS
    # -----------------------------

    def initialize(self):

        # Create AimToDirection command using calculated angle
        self.aimCommand = AimToDirection(
            self.getTargetAngle,
            self.drivetrain
        )

        self.aimCommand.initialize()

        # Calculate shooter angle
        shooter_angle = self.calculateShooterAngle()

        # Send angle to shooter subsystem
        self.shooter.setAngle(shooter_angle)

    def execute(self):
        self.aimCommand.execute()

    def end(self, interrupted):
        self.aimCommand.end(interrupted)

    def isFinished(self):
        return self.aimCommand.isFinished()