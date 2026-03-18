from rev import SparkMax, SparkLowLevel, SparkBaseConfig, ResetMode, PersistMode

from constants import HopperConstants
from commands2 import Subsystem
from wpilib import SmartDashboard

class Hopper(Subsystem):
    def __init__(self):
        super().__init__()

        self.roller_motor = SparkMax(
            HopperConstants.Roller_Motor_CAN_ID,
            SparkLowLevel.MotorType.kBrushless)
        self.roller_motor.setInverted(HopperConstants.Roller_Motor_Inverted)

        roller_motor_config = SparkBaseConfig()
        roller_motor_config.closedLoop.pid(
            HopperConstants.roller_motor_P,
            HopperConstants.roller_motor_I,
            HopperConstants.roller_motor_D
        )
        roller_motor_config.closedLoop.velocityFF(HopperConstants.roller_motor_F)

        self.roller_motor.configure(
            roller_motor_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)

        self.roller_pid = self.roller_motor.getClosedLoopController()

        self.roller_encoder = self.roller_motor.getEncoder()

        self.linear_motor = SparkMax(
            HopperConstants.Linear_Motor_CAN_ID,
            SparkLowLevel.MotorType.kBrushless
        )
        self.linear_motor.setInverted(HopperConstants.Linear_Motor_Inverted)

        linear_motor_config = SparkBaseConfig()
        linear_motor_config.closedLoop.pid(
            HopperConstants.linear_motor_P,
            HopperConstants.linear_motor_I,
            HopperConstants.linear_motor_D,
        )
        linear_motor_config.closedLoop.velocityFF(HopperConstants.linear_motor_F)

        linear_motor_config.closedLoop.outputRange(
            HopperConstants.linear_motor_min_speed,
            HopperConstants.linear_motor_max_speed
        )

        self.linear_motor.configure(
            linear_motor_config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.linear_pid = self.linear_motor.getClosedLoopController()

        self.linear_encoder = self.linear_motor.getEncoder()

    def extend_hopper(self):
        print("Extending hopper")
        self.linear_pid.setReference(HopperConstants.extend_hopper_final_pos, SparkMax.ControlType.kPosition)

    def retract_hopper(self):
        print("Retracting hopper")
        self.linear_pid.setReference(HopperConstants.retract_hopper_final_pos, SparkMax.ControlType.kPosition)

    def periodic(self) -> None:
        # Linear Position
        position = self.get_position()
        SmartDashboard.putNumber("Hopper Position", position)

        velocity = self.roller_encoder.getVelocity()
        SmartDashboard.putNumber("Hopper Velocity", velocity)

    def hopper_motor_spin_inwards(self):
        self.roller_pid.setReference(HopperConstants.inwards_spin_velocity, SparkMax.ControlType.kVelocity)

    def hopper_motor_spin_outwards(self):
        self.roller_pid.setReference(HopperConstants.outwards_spin_velocity, SparkMax.ControlType.kVelocity)

    def get_position(self):
        return self.linear_encoder.getPosition()

    def stop_linear_motor(self):
        self.linear_motor.stopMotor()

    def stop_rolling_motor(self):
        self.roller_motor.stopMotor()