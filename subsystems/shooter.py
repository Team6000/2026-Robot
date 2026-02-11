import math
from rev import SparkMax, SparkLowLevel, SparkBaseConfig, ResetMode, PersistMode
from constants import ShooterConstants
from commands2 import Subsystem

class Shooter(Subsystem):
    def __init__(self):
        """
        Shooter subsystem controlling a single flywheel motor.
        PID/FF are configured in SparkBaseConfig for closed-loop velocity control.
        """
        super().__init__()

        # Motor setup
        self.motor = SparkMax(
            ShooterConstants.Shooting_Motor_CAN_ID,
            SparkLowLevel.MotorType.kBrushless
        )
        self.motor.setInverted(ShooterConstants.Shooting_Motor_Inverted)

        # Encoder reference
        self.encoder = self.motor.getEncoder()

        # Configure closed-loop PID
        shooter_config = SparkBaseConfig()
        shooter_config.closedLoop.pid(
            ShooterConstants.shooter_P,
            ShooterConstants.shooter_I,
            ShooterConstants.shooter_D
        )
        shooter_config.closedLoop.velocityFF(ShooterConstants.shooter_F)

        # Apply configuration to motor
        self.motor.configure(
            shooter_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        # Store closed-loop controller once
        self.controller = self.motor.getClosedLoopController()

        # Target RPM
        self.target_rpm = 0.0

    def calculateShooterSpeed(self, pos_x: float):
        """
        Compute required wheel RPM to hit a target at horizontal distance pos_x.
        Uses simple projectile motion physics.
        """
        # TODO: CHANGE TO DISTANCE FORMULA FOR POSE

        # Avoid division by zero
        if pos_x == 0:
            self.target_rpm = 0.0
            return

        launch_angle = ShooterConstants.launch_angle
        hub_height = ShooterConstants.hub_height
        robot_height = ShooterConstants.robot_height
        wheel_circ = ShooterConstants.wheel_circumference

        # Calculate initial linear velocity
        init_velocity = math.sqrt(
            (9.8 * pos_x ** 2) /
            (2 * (math.cos(launch_angle) ** 2) *
             (pos_x * math.tan(launch_angle) - (hub_height - robot_height)))
        )

        # Convert linear velocity to wheel RPM
        self.target_rpm = (init_velocity / wheel_circ) * 60

    def setShooterRPM(self, rpm: float):
        """
        Command the motor to a specific RPM using closed-loop velocity control.
        """
        self.target_rpm = rpm
        self.controller.setSetpoint(self.target_rpm, SparkLowLevel.ControlType.kVelocity)

    def stop(self):
        """
        Stop the shooter motor.
        """
        self.target_rpm = 0.0
        self.controller.setSetpoint(0.0, SparkLowLevel.ControlType.kVelocity)

    def runCalculatedShooterSpeed(self, pos_x: float):
        self.calculateShooterSpeed(pos_x)
        self.controller.setSetpoint(self.target_rpm, SparkLowLevel.ControlType.kVelocity)


    def execute(self):
        """
        Command the shooter motor to the last calculated target RPM.
        """
        self.controller.setSetpoint(self.target_rpm, SparkLowLevel.ControlType.kVelocity)
