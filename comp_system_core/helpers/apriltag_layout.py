"""Local AprilTag field layout - replaces robotpy_apriltag.AprilTagFieldLayout.

WHY THIS EXISTS
---------------
robotpy-apriltag 2027.0.0a7 ships ONLY the detector and pose estimator:
    AprilTagDetection, AprilTagDetector, AprilTagPoseEstimate, AprilTagPoseEstimator
AprilTagFieldLayout, AprilTag and AprilTagField were all removed, and nothing in any other
a7 wheel replaces them (searched every one).  Since we ship our own field JSON every season
anyway - see 2026-rebuilt-welded_json, and the note about robotpy never having the current
field in time - owning the loader too costs us almost nothing.

The JSON is the standard WPILib layout format:
    {"tags": [{"ID": int, "pose": {"translation": {x,y,z},
                                   "rotation": {"quaternion": {W,X,Y,Z}}}}, ...],
     "field": {"length": float, "width": float}}

Only what the robot code actually uses is implemented: get_tag_pose() and get_tags().
"""

import json
from pathlib import Path

from wpimath import Pose3d, Quaternion, Rotation3d, Translation3d


class AprilTag:
    """One tag.  `ID` keeps WPILib's spelling because that is the JSON key and what
    callers already read (vision_sim iterates tags and reads tag.ID)."""

    __slots__ = ("ID", "pose")

    def __init__(self, tag_id: int, pose: Pose3d):
        self.ID = tag_id
        self.pose = pose

    def __repr__(self) -> str:
        return f"AprilTag(ID={self.ID}, pose={self.pose})"


class AprilTagFieldLayout:
    """Field layout loaded from a WPILib-format tag JSON."""

    def __init__(self, path: str | Path):
        self._path = Path(path)
        data = json.loads(self._path.read_text(encoding="utf-8"))

        self._tags: dict[int, AprilTag] = {}
        for entry in data["tags"]:
            t = entry["pose"]["translation"]
            q = entry["pose"]["rotation"]["quaternion"]
            pose = Pose3d(
                Translation3d(t["x"], t["y"], t["z"]),
                # WPILib's Quaternion is (w, x, y, z) - the JSON spells the keys in caps.
                Rotation3d(Quaternion(q["W"], q["X"], q["Y"], q["Z"])),
            )
            self._tags[int(entry["ID"])] = AprilTag(int(entry["ID"]), pose)

        field = data.get("field", {})
        self.field_length = field.get("length", 0.0)
        self.field_width = field.get("width", 0.0)

    def get_tag_pose(self, tag_id: int) -> Pose3d | None:
        """Pose of `tag_id`, or None if this layout does not contain it.

        Returning None rather than raising is load-bearing: swerve.py and swerve_sim.py both
        use `get_tag_pose(id) is None` to reject a training tag that is not part of the field.
        """
        tag = self._tags.get(int(tag_id))
        return tag.pose if tag is not None else None

    def get_tags(self) -> list[AprilTag]:
        """Every tag in the layout, ascending by ID."""
        return [self._tags[k] for k in sorted(self._tags)]

    def __len__(self) -> int:
        return len(self._tags)

    def __repr__(self) -> str:
        return f"AprilTagFieldLayout({self._path.name!r}, {len(self._tags)} tags)"
