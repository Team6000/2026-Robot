from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

# Load built-in 2026 field
layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)

HUB_WIDTH = inchesToMeters(47.0)

field_length = layout.getFieldLength()
field_width = layout.getFieldWidth()

tag_pose = layout.getTagPose(26).toPose2d()

blue = Pose2d(
    tag_pose.X() + HUB_WIDTH / 2.0,
    field_width / 2.0,
    tag_pose.rotation()
)

red = Pose2d(
    field_length - blue.X(),
    blue.Y(),
    blue.rotation().rotateBy(Rotation2d.fromDegrees(180))
)

print("Blue:", blue)
print("Red:", red)