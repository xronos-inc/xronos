# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import dataclass, field
from enum import Enum
from typing import ClassVar, Dict, Iterator, List, Optional, Tuple


class Actuator(Enum):
    """Enum for motor identifiers representing the degrees of freedom."""

    ROT_Z_DEG = 1
    PITCH1_DEG = 2
    PITCH2_DEG = 3
    PITCH3_DEG = 4
    GRIPPER_ROT_DEG = 5
    GRIPPER_DEG = 6


@dataclass(frozen=True)
class Pose:
    """Pose of a 6 DOF robot arm with actuator goal positions."""

    in_position_tolerance_deg: ClassVar[int] = 5

    positions: Dict[Actuator, int] = field(default_factory=dict[Actuator, int])
    name: Optional[str] = field(default="")

    def validate(self) -> None:
        for actuator in self.positions:
            Pose.validate_actuator(actuator, self.positions[actuator])

    @staticmethod
    def validate_actuator(actuator: Actuator, position_deg: int) -> None:
        """Validates the goal position for each actuator."""
        if actuator == Actuator.GRIPPER_ROT_DEG:
            if not (0 <= position_deg <= 270):  # noqa: PLR2004
                raise ValueError(
                    f"{actuator.name} position must be between 0 and 270 degrees"
                )
        elif not (0 <= position_deg <= 180):  # noqa: PLR2004
            raise ValueError(
                f"{actuator.name} position must be between 0 and 180 degrees"
            )

    def delta(self, x: "Pose") -> "Pose":
        """Return actuators whose current pose differs from the comparison pose.

        The returned list contains actuators whose position differs by more than
        `Pose.in_position_tolerance_deg`.
        """
        filtered_positions = {
            actuator: position
            for actuator, position in (self - x).positions.items()
            if actuator in self.positions
            and abs(position) > Pose.in_position_tolerance_deg
        }
        return Pose(positions=filtered_positions, name="Δ")

    def __sub__(self, other: "Pose") -> "Pose":
        """Subtracts the positions of another pose from the current pose."""
        result_positions = {
            actuator: self.positions.get(actuator, 0) - other.positions.get(actuator, 0)
            for actuator in set(self.positions) | set(other.positions)
        }
        return Pose(positions=result_positions)

    def __getitem__(self, actuator: Actuator) -> int | None:
        """Index by actuator to return the goal pose (position in degrees)."""
        if actuator in self.positions:
            return self.positions[actuator]
        else:
            return None

    def __setitem__(self, actuator: Actuator, position_deg: int) -> None:
        """Set the goal position for the specified actuator."""
        self.positions[actuator] = position_deg

    def __iter__(self) -> Iterator[Tuple[int, int]]:
        """Iterate over actuator indeces and positions."""
        return (
            (actuator.value, position) for actuator, position in self.positions.items()
        )

    def __len__(self) -> int:
        """Number of actuators stored in the pose."""
        return len(self.positions)

    def __str__(self) -> str:
        """Formatted string for this pose."""
        return ", ".join(
            [f"{actuator.name}: {value}" for actuator, value in self.positions.items()]
            + ([f'"{self.name}"'] if self.name else [])
        )


class Trajectory:
    """A sequence of poses to be executed by the robot arm.

    Args:
    poses: A list of poses making up the trajectory.
    """

    def __init__(self, poses: List[Pose]):
        self._poses = poses
        self._iter = iter(self._poses)

    def next_pose(self) -> Pose | None:
        """Returns the next pose in the trajectory or None."""
        try:
            return next(self._iter)
        except StopIteration:
            return None
