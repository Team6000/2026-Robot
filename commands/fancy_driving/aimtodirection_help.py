from __future__ import annotations

import math
from wpimath.geometry import Rotation2d
from constants import AutoMovementConstants


class AimToDirectionConstants:
    kP = 0.002
    kUseSqrtControl = AutoMovementConstants.kUseSqrtControl
    kMinTurnSpeed = 0.03
    kAngleToleranceDegrees = 2.0
    kAngleVelocityToleranceDegreesPerSec = 50


class AimToDirectionHelper:
    """
    Helper class that calculates rotation speed needed to aim at a target angle.

    DOES NOT control drivetrain directly.
    Safe to use alongside driver controls.
    """

    def __init__(self, drivetrain, max_speed=1.0):
        self.drivetrain = drivetrain

        # Clamp speed
        if abs(max_speed) < 1.0:
            self.speed = abs(max_speed)
        else:
            self.speed = 1.0

    def _wrapAngle(self, degrees: float) -> float:
        """Wrap angle to [-180, 180]"""
        while degrees > 180:
            degrees -= 360
        while degrees < -180:
            degrees += 360
        return degrees

    def getTurnSpeed(self, targetDegrees: float) -> float:
        """
        Returns rotation speed (-1 to 1) to aim at targetDegrees.
        """

        currentDirection = self.drivetrain.getGyroHeading()
        targetDirection = Rotation2d.fromDegrees(targetDegrees)

        rotationRemaining = targetDirection - currentDirection
        degreesRemaining = self._wrapAngle(rotationRemaining.degrees())

        # Proportional control
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)

        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)

        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed

        # Prevent stalling
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed:
            turnSpeed = AimToDirectionConstants.kMinTurnSpeed

        # Apply direction
        if degreesRemaining > 0:
            return +turnSpeed
        else:
            return -turnSpeed

    def atTarget(self, targetDegrees: float) -> bool:
        """
        Returns True if robot is aimed at target within tolerance.
        """

        currentDirection = self.drivetrain.getGyroHeading()
        targetDirection = Rotation2d.fromDegrees(targetDegrees)

        error = self._wrapAngle((targetDirection - currentDirection).degrees())

        if abs(error) < AimToDirectionConstants.kAngleToleranceDegrees:
            turnVelocity = self.drivetrain.getTurnRateDegreesPerSec()
            if abs(turnVelocity) < AimToDirectionConstants.kAngleVelocityToleranceDegreesPerSec:
                return True

        return False