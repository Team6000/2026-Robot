import commands2

# TODO: ADD UNSTUCK FUNCTION REVERSES
class IntakeCommand(commands2.Command):
    def __init__(self, shooter, hopper):
        super().__init__()

        self.shooter = shooter
        self.hopper = hopper


    def intake(self):
        self.shooter.intake()
        self.hopper.hopper_motor_spin_outwards()

    def execute(self):
        self.intake()

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.hopper.stop_rolling_motor()