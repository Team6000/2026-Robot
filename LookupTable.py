from _ast import Dict

import numpy as np


class LookupTable:
    """
    Allows you to lookup points in a pre-calibrated table, using linear interpolation.
    Example:

```
    shooting_angle_table = LookupTable({
       1.0 : 80,
       3.0 : 60,
       12.0 : 45,
    })
    # ^^ when firing form 1.0 meters away, fire at an 80 degree angle, etc.
    ...

    dist = self.camera.distanceTo(self.redTarget)

    angle = shooting_angle_table.interpolate(dist)

    self.shooter.setAngle(angle)
```

    """
    def __init__(self, points: Dict[float, float]):
        points = sorted(points.items())
        self.x = np.array([x for x, y in points], dtype=np.float32)
        self.y = np.array([y for x, y in points], dtype=np.float32)

    def interpolate(self, x: float):
        return np.interp(x, self.x, self.y)