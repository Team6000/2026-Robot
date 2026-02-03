import wpilib
import math
import time
from rev import CANSparkMax, CANSparkMaxLowLevel
from constants import ShooterConstants

class Shooter(Subsystem):
    def __init__(self,
                 shooting_speed,
                 motor_CANId,
                 motor_inverted,
                 motorControllerType = SparkMax):

        self.motor = CANSparkMax(
            can_id,
            CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.encoder = self.motor.getEncoder()
        self.pid = self.motor.getPIDController()
        self.pid.setP(ShooterConstants.shooter_P)
        self.pid.setI(ShooterConstants.shooter_I)
        self.pid.setD(ShooterConstants.shooter_D)
        self.pid.setF(ShooterConstants.shooter_F)


    def calculateShooterSpeed(self, pos_x):


        init_velocity = math.sqrt(
            (9.8 * pos_x ** 2) /
            (2 * (math.cos(ShooterConstants.launch_angle) ** 2)
             * (pos_x * math.tan(ShooterConstants.launch_angle)
                - (ShooterConstants.hub_ht - ShooterConstants.robot_height)))
        )
        self.rpm = (init_velocity/ShooterConstants.wheel_circumference) * 60



    def execute(self):
        self.pid.setReference(self.rpm, CANSparkMax.ControlType.kVelocity)




