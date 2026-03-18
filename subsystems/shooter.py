import math
from rev import SparkMax, SparkLowLevel, SparkBaseConfig, ResetMode, PersistMode
from wpilib import SmartDashboard

from constants import ShooterConstants
from commands2 import Subsystem
import hopper
import LookupTable


"""
Add to shooter: the indexing

Shooter
    Shooter intaking
"""
class Shooter(Subsystem):
    def __init__(self):
        """
        Shooter subsystem controlling a single flywheel motor.
        PID/FF are configured in SparkBaseConfig for closed-loop velocity control.
        """
        super().__init__()
        self.hopper = hopper.Hopper()
        self.shooter_motor = SparkMax(
            ShooterConstants.Shooting_Motor_CAN_ID,
            SparkLowLevel.MotorType.kBrushless
        )
        self.shooter_motor.setInverted(ShooterConstants.Shooting_Motor_Inverted)

        shooter_motor_config = SparkBaseConfig()
        shooter_motor_config.closedLoop.pid(
            ShooterConstants.shooter_P,
            ShooterConstants.shooter_I,
            ShooterConstants.shooter_D,
        )
        shooter_motor_config.closedLoop.velocityFF(ShooterConstants.shooter_F)

        self.shooter_motor.configure(
            shooter_motor_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.target_rpm = 0
        self.shooter_pid = self.shooter_motor.getClosedLoopController()
        self.shooter_encoder = self.shooter_motor.getEncoder()

        self.indexer_motor = SparkMax(
            ShooterConstants.Indexer_Motor_CAN_ID,
            SparkLowLevel.MotorType.kBrushless
        )
        self.indexer_motor.setInverted(ShooterConstants.Indexer_Motor_Inverted)

        indexer_motor_config = SparkBaseConfig()
        indexer_motor_config.closedLoop.pid(
            ShooterConstants.shooter_index_F,
            ShooterConstants.shooter_index_I,
            ShooterConstants.shooter_index_D,
        )
        indexer_motor_config.closedLoop.velocityFF(ShooterConstants.shooter_index_F)

        self.indexer_motor.configure(
            indexer_motor_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.indexer_pid = self.indexer_motor.getClosedLoopController()
        self.indexer_encoder = self.indexer_motor.getEncoder()

    def periodic(self) -> None:
        SmartDashboard.putNumber("Shooter Velocity", (self.shooter_encoder.getVelocity()))
        SmartDashboard.putNumber('Target RPM', self.target_rpm)

    def calculateShooterSpeed(self, distance: float):

        """
        Set shooter RPM based on Epanov-style lookup table with interpolation.
        """

        # Lookup RPM from table (automatically interpolates between points)
        RECOMMENDED_SHOOTER_RPM_BY_DISTANCE = LookupTable({
        1.0: 2000,  # if distance is 1m, spin at 2000 rpm
        2.0: 3000,  # if distance is 2m, spin at 3000 rpm
        12.0: 6000,  # if distance is 12m, spin at 6000 rpm
            })
    # TODO: why error???

        rpm = RECOMMENDED_SHOOTER_RPM_BY_DISTANCE.interpolate(distance)

    # Set the target RPM
        self.target_rpm = rpm
    def setShooterRPM(self, rpm: float):
        """
        Command the motor to a specific RPM using closed-loop velocity control.
        """
        self.target_rpm = rpm
        self.shooter_pid.setSetpoint(self.target_rpm, SparkLowLevel.ControlType.kVelocity)


    def runCalculatedShooterSpeed(self, distance: float):
        self.calculateShooterSpeed(distance)
        self.shooter_pid.setSetpoint(self.target_rpm, SparkLowLevel.ControlType.kVelocity)
        self.indexer_motor.set(ShooterConstants.Indexer_motor_speed)

    def intake(self):
        self.indexer_pid.setReference(4000, SparkLowLevel.ControlType.kVelocity)
        self.hopper.hopper_motor_spin_inwards()


    def stop(self):
        self.target_rpm = 0.0
        self.shooter_pid.stop()
        self.indexer_pid.stop()