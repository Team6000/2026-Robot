from __future__ import annotations

import typing
import commands2
from commands.fancy_driving.aimtodirection_help import AimToDirectionHelper


class AimToDirectionConstants:
    kDriverOverrideDeadband = 0.1


class AimToDirection(commands2.Command):
    """
    Default-style drive command with auto-aim rotation.
    """

    def __init__(
        self,
        drivetrain,
        getX: typing.Callable[[], float],
        getY: typing.Callable[[], float],
        getRot: typing.Callable[[], float],
        targetDegrees: typing.Callable[[], float],
        fieldRelative: bool = True,
    ):
        super().__init__()

        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

        self.getX = getX
        self.getY = getY
        self.getRot = getRot
        self.targetDegrees = targetDegrees

        self.fieldRelative = fieldRelative

        self.aimHelper = AimToDirectionHelper(self.drivetrain)

    def execute(self):
        x = self.getX()
        y = self.getY()
        driver_rot = self.getRot()

        target = self.targetDegrees()

        if abs(driver_rot) > AimToDirectionConstants.kDriverOverrideDeadband:
            rot = driver_rot
        else:
            rot = self.aimHelper.getTurnSpeed(target)

        self.drivetrain.drive(x, y, rot, self.fieldRelative, True)

    def atTarget(self):
        return self.aimHelper.atTarget(self.targetDegrees())

    def end(self, interrupted: bool):
        self.drivetrain.drive(0, 0, 0, self.fieldRelative, True)

    def isFinished(self):
        return False