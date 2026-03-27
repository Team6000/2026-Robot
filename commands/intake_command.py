import commands2
from wpilib import Timer, SmartDashboard


class IntakeCommand(commands2.Command):
    def __init__(self, shooter, hopper):
        super().__init__()

        self.shooter = shooter
        self.hopper = hopper

        self.timer = Timer()
        self.state = "INTAKE"  # or "UNJAM"

    def initialize(self):
        self.state = "INTAKE"
        self.timer.stop()
        self.timer.reset()

    def intake(self):
        self.shooter.intake()
        self.hopper.hopper_motor_spin_outwards()

    def unstuck_intake(self):
        self.shooter.reverse_intake()

    def execute(self):
        shooter_vel = self.shooter.shooter_encoder.getVelocity()
        hopper_vel = self.hopper.roller_encoder.getVelocity()

        if self.state == "INTAKE":
            self.intake()

            if abs(shooter_vel) < 10 and abs(hopper_vel) > 1000:
                self.state = "UNJAM"
                self.timer.reset()
                self.timer.start()

        elif self.state == "UNJAM":
            self.unstuck_intake()

            if self.timer.get() > 0.5:
                self.timer.stop()
                self.state = "INTAKE"

        SmartDashboard.putString("Intake State", self.state)

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.hopper.stop_rolling_motor()

    def isFinished(self):
        return False