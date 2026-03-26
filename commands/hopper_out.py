import commands2

class ExtendHopper(commands2.Command):
    def __init__(self, hopper):
        super().__init__()
        self.hopper = hopper


    def execute(self):
        self.hopper.extend_hopper()

    def isFinished(self):
        return self.hopper.get_position() < -30

    def end(self, interrupted: bool):
        self.hopper.stop_linear_motor()