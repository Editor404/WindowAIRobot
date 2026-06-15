from __future__ import annotations

from dataclasses import dataclass

from coordinate import RobotPose, TargetCoordinate


@dataclass
class RobotController:
    current_pose: RobotPose = RobotPose(0.0, 0.0)
    position_tolerance_cm: float = 0.5

    def home(self) -> None:
        """Move to the window-frame origin and reset the coordinate estimate."""
        print("Homing robot to origin...")
        self.current_pose = RobotPose(0.0, 0.0)

    def move_to(self, target: TargetCoordinate) -> None:
        dx = target.x_cm - self.current_pose.x_cm
        dy = target.y_cm - self.current_pose.y_cm

        if abs(dx) <= self.position_tolerance_cm and abs(dy) <= self.position_tolerance_cm:
            print("Target is already within tolerance.")
            return

        print(f"Moving X by {dx:.2f} cm")
        print(f"Moving Y by {dy:.2f} cm")
        self.current_pose = RobotPose(target.x_cm, target.y_cm)

    def clean(self) -> None:
        print("Cleaning target area...")
