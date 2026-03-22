from __future__ import annotations

import math
from wpimath.geometry import Rotation2d
from constants import AutoMovementConstants


class AimToDirectionConstants:
    # Increased P so it actually responds
    kP = 0.02

    # Turn OFF sqrt unless you really need it
    kUseSqrtControl = False

    # Minimum speed ONLY when far away
    kMinTurnSpeed = 0.03
    kAngleToleranceDegrees = 2.0
    kAngleVelocityToleranceDegreesPerSec = 50

    # Distance where we stop enforcing minimum speed
    kMinSpeedCutoffDegrees = 5.0


class AimToDirectionHelper:
    """
    Helper class that calculates rotation speed needed to aim at a target angle.

    DOES NOT control drivetrain directly.
    Safe to use alongside driver controls.
    """

    def __init__(self, drivetrain, max_speed=1.0):
        self.drivetrain = drivetrain
        self.speed = min(abs(max_speed), 1.0)

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

        # ✅ HARD STOP when within tolerance
        if abs(degreesRemaining) < AimToDirectionConstants.kAngleToleranceDegrees:
            return 0.0

        # ✅ Proportional control
        turnSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)

        # Optional shaping (usually keep OFF)
        if AimToDirectionConstants.kUseSqrtControl:
            turnSpeed = math.sqrt(0.5 * turnSpeed)

        # ✅ Clamp to max speed
        turnSpeed = min(turnSpeed, self.speed)

        # ✅ Only enforce min speed when far away
        if abs(degreesRemaining) > AimToDirectionConstants.kMinSpeedCutoffDegrees:
            turnSpeed = max(turnSpeed, AimToDirectionConstants.kMinTurnSpeed)

        return turnSpeed if degreesRemaining > 0 else -turnSpeed

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