# A Primer on Robot Geometry and Math

The geometry classes the navigation code is built on, with the names they have in this
project (2027a7: everything is imported flat from `wpimath`, methods are snake_case, and
coordinates are properties). `comp_bot/docs/MATH_PRIMER.md` has the 2026 spellings.

```python
from wpimath import Pose2d, Rotation2d, Translation2d, Transform2d
```

## 1. Coordinate systems

### Field frame
- Fixed to the field. All poses from the pose estimator are in it, whichever alliance we
  are on ("blue origin").
- The origin `(0, 0)` is the corner of the field on the **right-hand end of the Blue
  Alliance wall**, as seen by a blue driver.
- **+X** points downfield, away from the Blue Alliance wall.
- **+Y** points to the left, as seen by a blue driver.
- **Angles** are counter-clockwise; 0 degrees points along +X.
- The field is 16.54 m by 8.07 m (`constants.FieldConstants`).

### Robot frame
- Moves with the robot. The origin is the robot's centre.
- **+X** is forward, **+Y** is to the robot's left, angles are counter-clockwise.
- Camera targets (distance, rotation) and robot-oriented driving are in this frame.

## 2. The objects

### `Translation2d(x, y)`
A point, or a vector. `b - a` is the vector **from a to b**.

| You want | Write |
| :--- | :--- |
| the coordinates | `t.x`, `t.y` (properties, no parentheses) |
| its length | `t.norm()` |
| distance to another point | `a.distance(b)` |
| its direction | `t.angle()` (a `Rotation2d`) |
| the same vector turned | `t.rotate_by(rotation)` |

### `Rotation2d`
An angle that wraps correctly: 10 degrees minus 350 degrees is 20 degrees, not -340.

```python
r = Rotation2d.from_degrees(90)     # or Rotation2d(math.pi / 2), in radians
r.degrees(), r.radians()            # these two are methods
```

### `Pose2d(x, y, rotation)`
Where the robot is and which way it faces.

```python
pose = swerve.get_pose()
pose.x, pose.y                      # properties
pose.translation(), pose.rotation() # methods
```

### `Transform2d(translation, rotation)`
"From here, go this far and turn this much", in the frame of the pose it is applied to.
`pose.transform_by(t)` is how a robot-frame offset becomes a field pose.

## 3. The calculations you will meet

### The vector to a target
```python
vector_to_target = target_pos - robot_pos          # Translation2d
distance = vector_to_target.norm()
```

### The angle to turn
```python
field_angle_to_target = vector_to_target.angle()            # in the field frame
angle_to_turn = field_angle_to_target - pose.rotation()     # what a rotation PID works on
```
`subsystems/targeting.py` does exactly this to aim at the hub, with the robot's position
moved ahead by its velocity times the ball's time of flight.

### A field vector seen from the robot
Rotate by the **negative** of the robot's heading:
```python
robot_relative = field_vector.rotate_by(-pose.rotation())
strafe = robot_relative.y      # positive: target is to the robot's left
```
The other direction (robot frame to field frame) rotates by the heading itself;
`targeting.py` does that to turn the measured chassis velocity into a field velocity.

### A camera target as a field pose
`Vision.get_vision_target_pose()` builds the target in the robot frame from the camera's
distance and angle, then calls `current_pose.transform_by(...)`.

### The other alliance
`helpers/apriltag_utils.py` has the two operations, and they are different:

- `mirror_for_alliance(pose)` rotates a blue pose 180 degrees about the field centre when
  we are red. This is the alliance flip.
- `auto_reflect_pose(...)` also reflects about the field's long centreline when the robot
  is in the top half, so one auto works from either side.
