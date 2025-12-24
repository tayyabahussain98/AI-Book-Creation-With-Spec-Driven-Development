<<<<<<< HEAD
#!/usr/bin/env python3
"""
Safety Monitor for VLA Robot System

This module implements safety monitoring for autonomous humanoid robots,
including multi-layer safety checks, emergency stop handling, and
workspace boundary enforcement.

Module 4, Chapter 5: Capstone - Safety Constraints for Autonomous Operation
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, List, Dict, Tuple, Callable
from abc import ABC, abstractmethod
import time
import threading


class SafetyLevel(Enum):
    """Safety violation levels."""
    NOMINAL = auto()      # Normal operation
    CAUTION = auto()      # Approach limit - monitor closely
    WARNING = auto()      # Violation detected - reduce speed
    CRITICAL = auto()     # Immediate stop required
    EMERGENCY = auto()    # Emergency stop triggered


class ViolationType(Enum):
    """Types of safety violations."""
    JOINT_LIMIT = auto()
    VELOCITY_LIMIT = auto()
    ACCELERATION_LIMIT = auto()
    FORCE_LIMIT = auto()
    WORKSPACE_BOUNDARY = auto()
    COLLISION = auto()
    HUMAN_PROXIMITY = auto()
    E_STOP_TRIGGERED = auto()
    TIMEOUT = auto()


@dataclass
class SafetyViolation:
    """A detected safety violation."""
    violation_type: ViolationType
    safety_level: SafetyLevel
    message: str
    timestamp: float = field(default_factory=time.time)
    joint_index: Optional[int] = None
    value: float = 0.0
    limit: float = 0.0
    resolved: bool = False


@dataclass
class JointLimits:
    """Joint position limits."""
    min_position: float
    max_position: float
    min_velocity: float = -2.0
    max_velocity: float = 2.0
    min_acceleration: float = -5.0
    max_acceleration: float = 5.0
    max_effort: float = 50.0


@dataclass
class WorkspaceBoundary:
    """3D workspace boundary."""
    center: Tuple[float, float, float]
    radius: float  # Spherical boundary radius
    min_z: float = 0.0  # Floor constraint
    max_z: float = 2.0  # Height limit


@dataclass
class SafetyConfig:
    """Safety monitoring configuration."""
    joint_limits: List[JointLimits] = field(default_factory=list)
    workspace: Optional[WorkspaceBoundary] = None
    human_safety_distance: float = 0.5
    check_interval: float = 0.01  # 10ms real-time check
    velocity_scaling: Dict[SafetyLevel, float] = field(default_factory=dict)
    force_safety_factor: float = 0.8  # Use 80% of rated capacity

    def __post_init__(self):
        # Default velocity scaling by safety level
        if not self.velocity_scaling:
            self.velocity_scaling = {
                SafetyLevel.NOMINAL: 1.0,
                SafetyLevel.CAUTION: 0.75,
                SafetyLevel.WARNING: 0.5,
                SafetyLevel.CRITICAL: 0.25,
                SafetyLevel.EMERGENCY: 0.0,
            }


class SafetyMonitor:
    """
    Multi-layer safety monitoring system for VLA robots.

    Provides comprehensive safety checking including:
    - Joint limit enforcement
    - Velocity and acceleration limits
    - Workspace boundary constraints
    - Human proximity detection
    - Emergency stop handling
    """

    def __init__(self, config: SafetyConfig):
        self.config = config
        self.current_safety_level = SafetyLevel.NOMINAL
        self.active_violations: List[SafetyViolation] = []
        self.violation_history: List[SafetyViolation] = []
        self.e_stop_triggered = False

        # Threading for concurrent monitoring
        self._monitoring = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Callbacks for external integration
        self.on_violation: Optional[Callable[[SafetyViolation], None]] = None
        self.on_safety_level_change: Optional[Callable[[SafetyLevel], None]] = None
        self.on_e_stop: Optional[Callable[[], None]] = None

        # State for monitoring
        self._last_joint_positions: List[float] = []
        self._last_check_time: float = 0.0

    def start(self):
        """Start the safety monitoring system."""
        self._monitoring = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        print("Safety monitor started")

    def stop(self):
        """Stop the safety monitoring system."""
        self._monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
        print("Safety monitor stopped")

    def trigger_e_stop(self, reason: str = "Manual trigger"):
        """Trigger emergency stop."""
        with self._lock:
            self.e_stop_triggered = True
            self.current_safety_level = SafetyLevel.EMERGENCY

        violation = SafetyViolation(
            violation_type=ViolationType.E_STOP_TRIGGERED,
            safety_level=SafetyLevel.EMERGENCY,
            message=f"Emergency stop: {reason}"
        )
        self._record_violation(violation)

        if self.on_e_stop:
            self.on_e_stop()

        print(f"EMERGENCY STOP: {reason}")

    def reset_e_stop(self, reason: str = "Manual reset"):
        """Reset emergency stop."""
        with self._lock:
            self.e_stop_triggered = False
            self.current_safety_level = SafetyLevel.NOMINAL

        print(f"Emergency stop reset: {reason}")

    def check_joint_positions(
        self,
        positions: List[float]
    ) -> List[SafetyViolation]:
        """
        Check joint positions against limits.

        Args:
            positions: Current joint positions

        Returns:
            List of violations (empty if none)
        """
        violations = []

        for i, pos in enumerate(positions):
            if i >= len(self.config.joint_limits):
                continue

            limits = self.config.joint_limits[i]

            # Check hard limits
            if pos < limits.min_position:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} below minimum: {pos:.3f} < {limits.min_position:.3f}",
                    joint_index=i,
                    value=pos,
                    limit=limits.min_position
                ))
            elif pos > limits.max_position:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} above maximum: {pos:.3f} > {limits.max_position:.3f}",
                    joint_index=i,
                    value=pos,
                    limit=limits.max_position
                ))

            # Check soft limits (approach limits)
            margin = (limits.max_position - limits.min_position) * 0.05
            if pos < limits.min_position + margin:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CAUTION,
                    message=f"Joint {i} approaching minimum",
                    joint_index=i,
                    value=pos,
                    limit=limits.min_position
                ))
            elif pos > limits.max_position - margin:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CAUTION,
                    message=f"Joint {i} approaching maximum",
                    joint_index=i,
                    value=pos,
                    limit=limits.max_position
                ))

        return violations

    def check_velocities(
        self,
        velocities: List[float],
        timestamp: float
    ) -> List[SafetyViolation]:
        """
        Check joint velocities against limits.

        Args:
            velocities: Current joint velocities
            timestamp: Current timestamp

        Returns:
            List of violations (empty if none)
        """
        violations = []

        # Calculate accelerations if we have previous data
        accelerations = []
        if self._last_joint_positions and self._last_check_time > 0:
            dt = timestamp - self._last_check_time
            if dt > 0:
                accelerations = [
                    (v - p) / dt
                    for v, p in zip(velocities, self._last_joint_positions)
                ]

        for i, vel in enumerate(velocities):
            if i >= len(self.config.joint_limits):
                continue

            limits = self.config.joint_limits[i]

            # Check velocity limits
            abs_vel = abs(vel)
            if abs_vel > limits.max_velocity:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.VELOCITY_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} velocity exceeded: {abs_vel:.3f} > {limits.max_velocity:.3f}",
                    joint_index=i,
                    value=abs_vel,
                    limit=limits.max_velocity
                ))
            elif abs_vel > limits.max_velocity * 0.8:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.VELOCITY_LIMIT,
                    safety_level=SafetyLevel.WARNING,
                    message=f"Joint {i} velocity approaching limit",
                    joint_index=i,
                    value=abs_vel,
                    limit=limits.max_velocity
                ))

            # Check acceleration limits
            if i < len(accelerations):
                acc = accelerations[i]
                abs_acc = abs(acc)
                max_acc = abs(limits.max_acceleration)

                if abs_acc > max_acc:
                    violations.append(SafetyViolation(
                        violation_type=ViolationType.ACCELERATION_LIMIT,
                        safety_level=SafetyLevel.WARNING,
                        message=f"Joint {i} acceleration exceeded: {abs_acc:.3f} > {max_acc:.3f}",
                        joint_index=i,
                        value=abs_acc,
                        limit=max_acc
                    ))

        # Update state
        self._last_joint_positions = list(velocities)
        self._last_check_time = timestamp

        return violations

    def check_workspace(
        self,
        cartesian_position: Tuple[float, float, float]
    ) -> List[SafetyViolation]:
        """
        Check if position is within workspace boundaries.

        Args:
            cartesian_position: (x, y, z) position

        Returns:
            List of violations (empty if none)
        """
        violations = []
        workspace = self.config.workspace

        if not workspace:
            return violations

        x, y, z = cartesian_position

        # Check spherical boundary
        dist_from_center = (
            (x - workspace.center[0])**2 +
            (y - workspace.center[1])**2 +
            (z - workspace.center[2])**2
        )**0.5

        if dist_from_center > workspace.radius:
            violations.append(SafetyViolation(
                violation_type=ViolationType.WORKSPACE_BOUNDARY,
                safety_level=SafetyLevel.CRITICAL,
                message=f"Position outside workspace: distance {dist_from_center:.3f} > {workspace.radius:.3f}",
                value=dist_from_center,
                limit=workspace.radius
            ))

        # Check z boundaries
        if z < workspace.min_z:
            violations.append(SafetyViolation(
                violation_type=ViolationType.WORKSPACE_BOUNDARY,
                safety_level=SafetyLevel.CRITICAL,
                message=f"Position below minimum height: {z:.3f} < {workspace.min_z:.3f}",
                value=z,
                limit=workspace.min_z
            ))

        if z > workspace.max_z:
            violations.append(SafetyViolation(
                violation_type=ViolationType.WORKSPACE_BOUNDARY,
                safety_level=SafetyLevel.CRITICAL,
                message=f"Position above maximum height: {z:.3f} > {workspace.max_z:.3f}",
                value=z,
                limit=workspace.max_z
            ))

        return violations

    def check_human_proximity(
        self,
        robot_position: Tuple[float, float, float],
        human_positions: List[Tuple[float, float, float]]
    ) -> List[SafetyViolation]:
        """
        Check distance to humans in the environment.

        Args:
            robot_position: Current robot end-effector position
            human_positions: List of detected human positions

        Returns:
            List of violations (empty if none)
        """
        violations = []

        for i, human_pos in enumerate(human_positions):
            distance = (
                (robot_position[0] - human_pos[0])**2 +
                (robot_position[1] - human_pos[1])**2 +
                (robot_position[2] - human_pos[2])**2
            )**0.5

            if distance < self.config.human_safety_distance * 0.5:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.HUMAN_PROXIMITY,
                    safety_level=SafetyLevel.EMERGENCY,
                    message=f"Human too close: {distance:.3f}m (safety: {self.config.human_safety_distance}m)",
                    value=distance,
                    limit=self.config.human_safety_distance
                ))
            elif distance < self.config.human_safety_distance:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.HUMAN_PROXIMITY,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Human in caution zone: {distance:.3f}m",
                    value=distance,
                    limit=self.config.human_safety_distance
                ))

        return violations

    def check_force(
        self,
        measured_forces: List[float]
    ) -> List[SafetyViolation]:
        """
        Check measured forces against limits.

        Args:
            measured_forces: Current force measurements per joint

        Returns:
            List of violations (empty if none)
        """
        violations = []

        for i, force in enumerate(measured_forces):
            if i >= len(self.config.joint_limits):
                continue

            limit = self.config.joint_limits[i].max_effort * self.config.force_safety_factor
            abs_force = abs(force)

            if abs_force > limit:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.FORCE_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} force exceeded: {abs_force:.1f}N > {limit:.1f}N",
                    joint_index=i,
                    value=abs_force,
                    limit=limit
                ))

        return violations

    def check_all(
        self,
        joint_positions: List[float],
        joint_velocities: List[float],
        cartesian_position: Tuple[float, float, float],
        measured_forces: List[float] = None,
        human_positions: List[Tuple[float, float, float]] = None,
        timestamp: float = None
    ) -> Tuple[SafetyLevel, List[SafetyViolation]]:
        """
        Perform comprehensive safety check.

        Args:
            joint_positions: Current joint positions
            joint_velocities: Current joint velocities
            cartesian_position: Current end-effector position
            measured_forces: Optional force measurements
            human_positions: Optional list of human positions
            timestamp: Optional timestamp (uses time.time() if not provided)

        Returns:
            Tuple of (overall safety level, list of violations)
        """
        if timestamp is None:
            timestamp = time.time()

        all_violations = []

        # Check each aspect
        all_violations.extend(self.check_joint_positions(joint_positions))
        all_violations.extend(self.check_velocities(joint_velocities, timestamp))
        all_violations.extend(self.check_workspace(cartesian_position))

        if measured_forces:
            all_violations.extend(self.check_force(measured_forces))

        if human_positions:
            all_violations.extend(self.check_human_proximity(cartesian_position, human_positions))

        # Determine overall safety level
        if not all_violations:
            return SafetyLevel.NOMINAL, []

        # Find highest severity violation
        highest_level = SafetyLevel.NOMINAL
        for violation in all_violations:
            if violation.safety_level.value > highest_level.value:
                highest_level = violation.safety_level

        return highest_level, all_violations

    def get_velocity_scale(self, safety_level: SafetyLevel) -> float:
        """Get velocity scaling factor based on safety level."""
        return self.config.velocity_scaling.get(safety_level, 0.0)

    def get_status(self) -> Dict:
        """Get current safety monitor status."""
        with self._lock:
            return {
                'safety_level': self.current_safety_level.name,
                'e_stop_triggered': self.e_stop_triggered,
                'active_violations': len(self.active_violations),
                'violation_history_count': len(self.violation_history),
                'nominal_velocity_scale': self.get_velocity_scale(SafetyLevel.NOMINAL)
            }

    def _record_violation(self, violation: SafetyViolation):
        """Record a safety violation."""
        with self._lock:
            self.active_violations.append(violation)
            self.violation_history.append(violation)

            # Notify callback
            if self.on_violation:
                self.on_violation(violation)

            # Update safety level if needed
            if violation.safety_level.value > self.current_safety_level.value:
                old_level = self.current_safety_level
                self.current_safety_level = violation.safety_level

                if self.on_safety_level_change:
                    self.on_safety_level_change(violation.safety_level)

    def _monitor_loop(self):
        """Background monitoring loop."""
        while self._monitoring:
            try:
                time.sleep(self.config.check_interval)

                # In a real system, would read actual sensor data
                # For now, just check if we're still running
                if not self._monitoring:
                    break

            except Exception as e:
                print(f"Error in safety monitor loop: {e}")


class SafetyGuard:
    """
    Safety guard that can be used to wrap operations with safety checks.
    """

    def __init__(self, safety_monitor: SafetyMonitor):
        self.safety_monitor = safety_monitor

    def check_before_motion(
        self,
        planned_trajectory: List[Dict],
        current_positions: List[float]
    ) -> Tuple[bool, List[str]]:
        """
        Check if a planned trajectory is safe to execute.

        Args:
            planned_trajectory: List of trajectory waypoints
            current_positions: Current joint positions

        Returns:
            Tuple of (is_safe, list of violations)
        """
        violations = []

        for i, waypoint in enumerate(planned_trajectory):
            # Check joint positions
            joint_positions = waypoint.get('joint_positions', current_positions)
            joint_velocities = waypoint.get('velocities', [0.0] * len(joint_positions))
            cartesian = waypoint.get('cartesian_position', (0, 0, 0))

            level, waypoint_violations = self.safety_monitor.check_all(
                joint_positions,
                joint_velocities,
                cartesian
            )

            violations.extend([v.message for v in waypoint_violations])

            # Check for critical violations
            if level == SafetyLevel.CRITICAL:
                return False, violations

            # Check for trajectory jumps
            if i == 0 and current_positions:
                for j in range(min(len(joint_positions), len(current_positions))):
                    jump = abs(joint_positions[j] - current_positions[j])
                    if jump > 0.5:  # More than ~30 degrees
                        violations.append(
                            f"Large jump detected at joint {j}: {jump:.3f} rad"
                        )

        return len(violations) == 0, violations


def create_default_config() -> SafetyConfig:
    """Create default safety configuration for a 7-DOF robot arm."""
    joint_limits = []

    # Typical 7-DOF arm joint limits (in radians)
    joint_configs = [
        (-3.14, 3.14, 2.0, 5.0, 30.0),   # Shoulder yaw
        (-2.5, 2.5, 1.5, 3.0, 25.0),     # Shoulder pitch
        (-3.0, 3.0, 2.0, 5.0, 20.0),     # Elbow pitch
        (-2.5, 2.5, 2.0, 5.0, 15.0),     # Elbow roll
        (-3.14, 3.14, 2.5, 6.0, 12.0),   # Wrist pitch
        (-2.0, 2.0, 2.5, 6.0, 10.0),     # Wrist roll
        (-3.14, 3.14, 3.0, 8.0, 8.0),    # Hand
    ]

    for min_pos, max_pos, max_vel, max_acc, max_effort in joint_configs:
        joint_limits.append(JointLimits(
            min_position=min_pos,
            max_position=max_pos,
            min_velocity=-max_vel,
            max_velocity=max_vel,
            min_acceleration=-max_acc,
            max_acceleration=max_acc,
            max_effort=max_effort
        ))

    workspace = WorkspaceBoundary(
        center=(0.5, 0.0, 0.5),
        radius=0.8,
        min_z=0.0,
        max_z=1.5
    )

    return SafetyConfig(
        joint_limits=joint_limits,
        workspace=workspace,
        human_safety_distance=0.5
    )


def main():
    """Demonstration of safety monitoring."""
    # Create safety configuration
    config = create_default_config()

    # Create safety monitor
    safety_monitor = SafetyMonitor(config)

    # Setup callbacks
    def on_violation(v: SafetyViolation):
        print(f"VIOLATION [{v.safety_level.name}]: {v.message}")

    def on_level_change(level: SafetyLevel):
        print(f"SAFETY LEVEL CHANGED: {level.name}")

    safety_monitor.on_violation = on_violation
    safety_monitor.on_safety_level_change = on_level_change

    # Start monitoring
    safety_monitor.start()

    # Test joint limit checking
    print("\n=== Testing Joint Limits ===")
    test_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Normal
    level, violations = safety_monitor.check_all(
        test_positions,
        [0.0] * 7,
        (0.5, 0.0, 0.5)
    )
    print(f"Normal positions - Safety level: {level.name}, Violations: {len(violations)}")

    # Test position near limit
    test_positions = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Near limit
    level, violations = safety_monitor.check_all(
        test_positions,
        [0.0] * 7,
        (0.5, 0.0, 0.5)
    )
    print(f"Near limit - Safety level: {level.name}, Violations: {len(violations)}")

    # Test position over limit
    test_positions = [3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Over limit
    level, violations = safety_monitor.check_all(
        test_positions,
        [0.0] * 7,
        (0.5, 0.0, 0.5)
    )
    print(f"Over limit - Safety level: {level.name}, Violations: {len(violations)}")

    # Test workspace boundary
    print("\n=== Testing Workspace Boundary ===")
    level, violations = safety_monitor.check_all(
        [0.0] * 7,
        [0.0] * 7,
        (2.0, 0.0, 0.5)  # Outside workspace
    )
    print(f"Outside workspace - Safety level: {level.name}, Violations: {len(violations)}")

    # Test human proximity
    print("\n=== Testing Human Proximity ===")
    level, violations = safety_monitor.check_all(
        [0.0] * 7,
        [0.0] * 7,
        (0.5, 0.0, 0.5),
        human_positions=[(0.55, 0.0, 0.5)]  # Human very close
    )
    print(f"Human close - Safety level: {level.name}, Violations: {len(violations)}")

    # Test velocity checking
    print("\n=== Testing Velocities ===")
    level, violations = safety_monitor.check_velocities(
        [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # High velocity
        time.time()
    )
    print(f"High velocity - Safety level: {level.name}, Violations: {len(violations)}")

    # Test emergency stop
    print("\n=== Testing Emergency Stop ===")
    safety_monitor.trigger_e_stop("Test emergency stop")
    status = safety_monitor.get_status()
    print(f"After E-stop - Status: {status}")

    # Reset and verify
    safety_monitor.reset_e_stop("Test complete")
    status = safety_monitor.get_status()
    print(f"After reset - Status: {status}")

    # Stop monitoring
    safety_monitor.stop()

    print("\n=== Summary ===")
    print(f"Total violations recorded: {len(safety_monitor.violation_history)}")
    print(f"Active violations: {len(safety_monitor.active_violations)}")


if __name__ == '__main__':
    main()
=======
#!/usr/bin/env python3
"""
Safety Monitor for VLA Robot System

This module implements safety monitoring for autonomous humanoid robots,
including multi-layer safety checks, emergency stop handling, and
workspace boundary enforcement.

Module 4, Chapter 5: Capstone - Safety Constraints for Autonomous Operation
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, List, Dict, Tuple, Callable
from abc import ABC, abstractmethod
import time
import threading


class SafetyLevel(Enum):
    """Safety violation levels."""
    NOMINAL = auto()      # Normal operation
    CAUTION = auto()      # Approach limit - monitor closely
    WARNING = auto()      # Violation detected - reduce speed
    CRITICAL = auto()     # Immediate stop required
    EMERGENCY = auto()    # Emergency stop triggered


class ViolationType(Enum):
    """Types of safety violations."""
    JOINT_LIMIT = auto()
    VELOCITY_LIMIT = auto()
    ACCELERATION_LIMIT = auto()
    FORCE_LIMIT = auto()
    WORKSPACE_BOUNDARY = auto()
    COLLISION = auto()
    HUMAN_PROXIMITY = auto()
    E_STOP_TRIGGERED = auto()
    TIMEOUT = auto()


@dataclass
class SafetyViolation:
    """A detected safety violation."""
    violation_type: ViolationType
    safety_level: SafetyLevel
    message: str
    timestamp: float = field(default_factory=time.time)
    joint_index: Optional[int] = None
    value: float = 0.0
    limit: float = 0.0
    resolved: bool = False


@dataclass
class JointLimits:
    """Joint position limits."""
    min_position: float
    max_position: float
    min_velocity: float = -2.0
    max_velocity: float = 2.0
    min_acceleration: float = -5.0
    max_acceleration: float = 5.0
    max_effort: float = 50.0


@dataclass
class WorkspaceBoundary:
    """3D workspace boundary."""
    center: Tuple[float, float, float]
    radius: float  # Spherical boundary radius
    min_z: float = 0.0  # Floor constraint
    max_z: float = 2.0  # Height limit


@dataclass
class SafetyConfig:
    """Safety monitoring configuration."""
    joint_limits: List[JointLimits] = field(default_factory=list)
    workspace: Optional[WorkspaceBoundary] = None
    human_safety_distance: float = 0.5
    check_interval: float = 0.01  # 10ms real-time check
    velocity_scaling: Dict[SafetyLevel, float] = field(default_factory=dict)
    force_safety_factor: float = 0.8  # Use 80% of rated capacity

    def __post_init__(self):
        # Default velocity scaling by safety level
        if not self.velocity_scaling:
            self.velocity_scaling = {
                SafetyLevel.NOMINAL: 1.0,
                SafetyLevel.CAUTION: 0.75,
                SafetyLevel.WARNING: 0.5,
                SafetyLevel.CRITICAL: 0.25,
                SafetyLevel.EMERGENCY: 0.0,
            }


class SafetyMonitor:
    """
    Multi-layer safety monitoring system for VLA robots.

    Provides comprehensive safety checking including:
    - Joint limit enforcement
    - Velocity and acceleration limits
    - Workspace boundary constraints
    - Human proximity detection
    - Emergency stop handling
    """

    def __init__(self, config: SafetyConfig):
        self.config = config
        self.current_safety_level = SafetyLevel.NOMINAL
        self.active_violations: List[SafetyViolation] = []
        self.violation_history: List[SafetyViolation] = []
        self.e_stop_triggered = False

        # Threading for concurrent monitoring
        self._monitoring = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Callbacks for external integration
        self.on_violation: Optional[Callable[[SafetyViolation], None]] = None
        self.on_safety_level_change: Optional[Callable[[SafetyLevel], None]] = None
        self.on_e_stop: Optional[Callable[[], None]] = None

        # State for monitoring
        self._last_joint_positions: List[float] = []
        self._last_check_time: float = 0.0

    def start(self):
        """Start the safety monitoring system."""
        self._monitoring = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        print("Safety monitor started")

    def stop(self):
        """Stop the safety monitoring system."""
        self._monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
        print("Safety monitor stopped")

    def trigger_e_stop(self, reason: str = "Manual trigger"):
        """Trigger emergency stop."""
        with self._lock:
            self.e_stop_triggered = True
            self.current_safety_level = SafetyLevel.EMERGENCY

        violation = SafetyViolation(
            violation_type=ViolationType.E_STOP_TRIGGERED,
            safety_level=SafetyLevel.EMERGENCY,
            message=f"Emergency stop: {reason}"
        )
        self._record_violation(violation)

        if self.on_e_stop:
            self.on_e_stop()

        print(f"EMERGENCY STOP: {reason}")

    def reset_e_stop(self, reason: str = "Manual reset"):
        """Reset emergency stop."""
        with self._lock:
            self.e_stop_triggered = False
            self.current_safety_level = SafetyLevel.NOMINAL

        print(f"Emergency stop reset: {reason}")

    def check_joint_positions(
        self,
        positions: List[float]
    ) -> List[SafetyViolation]:
        """
        Check joint positions against limits.

        Args:
            positions: Current joint positions

        Returns:
            List of violations (empty if none)
        """
        violations = []

        for i, pos in enumerate(positions):
            if i >= len(self.config.joint_limits):
                continue

            limits = self.config.joint_limits[i]

            # Check hard limits
            if pos < limits.min_position:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} below minimum: {pos:.3f} < {limits.min_position:.3f}",
                    joint_index=i,
                    value=pos,
                    limit=limits.min_position
                ))
            elif pos > limits.max_position:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} above maximum: {pos:.3f} > {limits.max_position:.3f}",
                    joint_index=i,
                    value=pos,
                    limit=limits.max_position
                ))

            # Check soft limits (approach limits)
            margin = (limits.max_position - limits.min_position) * 0.05
            if pos < limits.min_position + margin:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CAUTION,
                    message=f"Joint {i} approaching minimum",
                    joint_index=i,
                    value=pos,
                    limit=limits.min_position
                ))
            elif pos > limits.max_position - margin:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.JOINT_LIMIT,
                    safety_level=SafetyLevel.CAUTION,
                    message=f"Joint {i} approaching maximum",
                    joint_index=i,
                    value=pos,
                    limit=limits.max_position
                ))

        return violations

    def check_velocities(
        self,
        velocities: List[float],
        timestamp: float
    ) -> List[SafetyViolation]:
        """
        Check joint velocities against limits.

        Args:
            velocities: Current joint velocities
            timestamp: Current timestamp

        Returns:
            List of violations (empty if none)
        """
        violations = []

        # Calculate accelerations if we have previous data
        accelerations = []
        if self._last_joint_positions and self._last_check_time > 0:
            dt = timestamp - self._last_check_time
            if dt > 0:
                accelerations = [
                    (v - p) / dt
                    for v, p in zip(velocities, self._last_joint_positions)
                ]

        for i, vel in enumerate(velocities):
            if i >= len(self.config.joint_limits):
                continue

            limits = self.config.joint_limits[i]

            # Check velocity limits
            abs_vel = abs(vel)
            if abs_vel > limits.max_velocity:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.VELOCITY_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} velocity exceeded: {abs_vel:.3f} > {limits.max_velocity:.3f}",
                    joint_index=i,
                    value=abs_vel,
                    limit=limits.max_velocity
                ))
            elif abs_vel > limits.max_velocity * 0.8:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.VELOCITY_LIMIT,
                    safety_level=SafetyLevel.WARNING,
                    message=f"Joint {i} velocity approaching limit",
                    joint_index=i,
                    value=abs_vel,
                    limit=limits.max_velocity
                ))

            # Check acceleration limits
            if i < len(accelerations):
                acc = accelerations[i]
                abs_acc = abs(acc)
                max_acc = abs(limits.max_acceleration)

                if abs_acc > max_acc:
                    violations.append(SafetyViolation(
                        violation_type=ViolationType.ACCELERATION_LIMIT,
                        safety_level=SafetyLevel.WARNING,
                        message=f"Joint {i} acceleration exceeded: {abs_acc:.3f} > {max_acc:.3f}",
                        joint_index=i,
                        value=abs_acc,
                        limit=max_acc
                    ))

        # Update state
        self._last_joint_positions = list(velocities)
        self._last_check_time = timestamp

        return violations

    def check_workspace(
        self,
        cartesian_position: Tuple[float, float, float]
    ) -> List[SafetyViolation]:
        """
        Check if position is within workspace boundaries.

        Args:
            cartesian_position: (x, y, z) position

        Returns:
            List of violations (empty if none)
        """
        violations = []
        workspace = self.config.workspace

        if not workspace:
            return violations

        x, y, z = cartesian_position

        # Check spherical boundary
        dist_from_center = (
            (x - workspace.center[0])**2 +
            (y - workspace.center[1])**2 +
            (z - workspace.center[2])**2
        )**0.5

        if dist_from_center > workspace.radius:
            violations.append(SafetyViolation(
                violation_type=ViolationType.WORKSPACE_BOUNDARY,
                safety_level=SafetyLevel.CRITICAL,
                message=f"Position outside workspace: distance {dist_from_center:.3f} > {workspace.radius:.3f}",
                value=dist_from_center,
                limit=workspace.radius
            ))

        # Check z boundaries
        if z < workspace.min_z:
            violations.append(SafetyViolation(
                violation_type=ViolationType.WORKSPACE_BOUNDARY,
                safety_level=SafetyLevel.CRITICAL,
                message=f"Position below minimum height: {z:.3f} < {workspace.min_z:.3f}",
                value=z,
                limit=workspace.min_z
            ))

        if z > workspace.max_z:
            violations.append(SafetyViolation(
                violation_type=ViolationType.WORKSPACE_BOUNDARY,
                safety_level=SafetyLevel.CRITICAL,
                message=f"Position above maximum height: {z:.3f} > {workspace.max_z:.3f}",
                value=z,
                limit=workspace.max_z
            ))

        return violations

    def check_human_proximity(
        self,
        robot_position: Tuple[float, float, float],
        human_positions: List[Tuple[float, float, float]]
    ) -> List[SafetyViolation]:
        """
        Check distance to humans in the environment.

        Args:
            robot_position: Current robot end-effector position
            human_positions: List of detected human positions

        Returns:
            List of violations (empty if none)
        """
        violations = []

        for i, human_pos in enumerate(human_positions):
            distance = (
                (robot_position[0] - human_pos[0])**2 +
                (robot_position[1] - human_pos[1])**2 +
                (robot_position[2] - human_pos[2])**2
            )**0.5

            if distance < self.config.human_safety_distance * 0.5:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.HUMAN_PROXIMITY,
                    safety_level=SafetyLevel.EMERGENCY,
                    message=f"Human too close: {distance:.3f}m (safety: {self.config.human_safety_distance}m)",
                    value=distance,
                    limit=self.config.human_safety_distance
                ))
            elif distance < self.config.human_safety_distance:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.HUMAN_PROXIMITY,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Human in caution zone: {distance:.3f}m",
                    value=distance,
                    limit=self.config.human_safety_distance
                ))

        return violations

    def check_force(
        self,
        measured_forces: List[float]
    ) -> List[SafetyViolation]:
        """
        Check measured forces against limits.

        Args:
            measured_forces: Current force measurements per joint

        Returns:
            List of violations (empty if none)
        """
        violations = []

        for i, force in enumerate(measured_forces):
            if i >= len(self.config.joint_limits):
                continue

            limit = self.config.joint_limits[i].max_effort * self.config.force_safety_factor
            abs_force = abs(force)

            if abs_force > limit:
                violations.append(SafetyViolation(
                    violation_type=ViolationType.FORCE_LIMIT,
                    safety_level=SafetyLevel.CRITICAL,
                    message=f"Joint {i} force exceeded: {abs_force:.1f}N > {limit:.1f}N",
                    joint_index=i,
                    value=abs_force,
                    limit=limit
                ))

        return violations

    def check_all(
        self,
        joint_positions: List[float],
        joint_velocities: List[float],
        cartesian_position: Tuple[float, float, float],
        measured_forces: List[float] = None,
        human_positions: List[Tuple[float, float, float]] = None,
        timestamp: float = None
    ) -> Tuple[SafetyLevel, List[SafetyViolation]]:
        """
        Perform comprehensive safety check.

        Args:
            joint_positions: Current joint positions
            joint_velocities: Current joint velocities
            cartesian_position: Current end-effector position
            measured_forces: Optional force measurements
            human_positions: Optional list of human positions
            timestamp: Optional timestamp (uses time.time() if not provided)

        Returns:
            Tuple of (overall safety level, list of violations)
        """
        if timestamp is None:
            timestamp = time.time()

        all_violations = []

        # Check each aspect
        all_violations.extend(self.check_joint_positions(joint_positions))
        all_violations.extend(self.check_velocities(joint_velocities, timestamp))
        all_violations.extend(self.check_workspace(cartesian_position))

        if measured_forces:
            all_violations.extend(self.check_force(measured_forces))

        if human_positions:
            all_violations.extend(self.check_human_proximity(cartesian_position, human_positions))

        # Determine overall safety level
        if not all_violations:
            return SafetyLevel.NOMINAL, []

        # Find highest severity violation
        highest_level = SafetyLevel.NOMINAL
        for violation in all_violations:
            if violation.safety_level.value > highest_level.value:
                highest_level = violation.safety_level

        return highest_level, all_violations

    def get_velocity_scale(self, safety_level: SafetyLevel) -> float:
        """Get velocity scaling factor based on safety level."""
        return self.config.velocity_scaling.get(safety_level, 0.0)

    def get_status(self) -> Dict:
        """Get current safety monitor status."""
        with self._lock:
            return {
                'safety_level': self.current_safety_level.name,
                'e_stop_triggered': self.e_stop_triggered,
                'active_violations': len(self.active_violations),
                'violation_history_count': len(self.violation_history),
                'nominal_velocity_scale': self.get_velocity_scale(SafetyLevel.NOMINAL)
            }

    def _record_violation(self, violation: SafetyViolation):
        """Record a safety violation."""
        with self._lock:
            self.active_violations.append(violation)
            self.violation_history.append(violation)

            # Notify callback
            if self.on_violation:
                self.on_violation(violation)

            # Update safety level if needed
            if violation.safety_level.value > self.current_safety_level.value:
                old_level = self.current_safety_level
                self.current_safety_level = violation.safety_level

                if self.on_safety_level_change:
                    self.on_safety_level_change(violation.safety_level)

    def _monitor_loop(self):
        """Background monitoring loop."""
        while self._monitoring:
            try:
                time.sleep(self.config.check_interval)

                # In a real system, would read actual sensor data
                # For now, just check if we're still running
                if not self._monitoring:
                    break

            except Exception as e:
                print(f"Error in safety monitor loop: {e}")


class SafetyGuard:
    """
    Safety guard that can be used to wrap operations with safety checks.
    """

    def __init__(self, safety_monitor: SafetyMonitor):
        self.safety_monitor = safety_monitor

    def check_before_motion(
        self,
        planned_trajectory: List[Dict],
        current_positions: List[float]
    ) -> Tuple[bool, List[str]]:
        """
        Check if a planned trajectory is safe to execute.

        Args:
            planned_trajectory: List of trajectory waypoints
            current_positions: Current joint positions

        Returns:
            Tuple of (is_safe, list of violations)
        """
        violations = []

        for i, waypoint in enumerate(planned_trajectory):
            # Check joint positions
            joint_positions = waypoint.get('joint_positions', current_positions)
            joint_velocities = waypoint.get('velocities', [0.0] * len(joint_positions))
            cartesian = waypoint.get('cartesian_position', (0, 0, 0))

            level, waypoint_violations = self.safety_monitor.check_all(
                joint_positions,
                joint_velocities,
                cartesian
            )

            violations.extend([v.message for v in waypoint_violations])

            # Check for critical violations
            if level == SafetyLevel.CRITICAL:
                return False, violations

            # Check for trajectory jumps
            if i == 0 and current_positions:
                for j in range(min(len(joint_positions), len(current_positions))):
                    jump = abs(joint_positions[j] - current_positions[j])
                    if jump > 0.5:  # More than ~30 degrees
                        violations.append(
                            f"Large jump detected at joint {j}: {jump:.3f} rad"
                        )

        return len(violations) == 0, violations


def create_default_config() -> SafetyConfig:
    """Create default safety configuration for a 7-DOF robot arm."""
    joint_limits = []

    # Typical 7-DOF arm joint limits (in radians)
    joint_configs = [
        (-3.14, 3.14, 2.0, 5.0, 30.0),   # Shoulder yaw
        (-2.5, 2.5, 1.5, 3.0, 25.0),     # Shoulder pitch
        (-3.0, 3.0, 2.0, 5.0, 20.0),     # Elbow pitch
        (-2.5, 2.5, 2.0, 5.0, 15.0),     # Elbow roll
        (-3.14, 3.14, 2.5, 6.0, 12.0),   # Wrist pitch
        (-2.0, 2.0, 2.5, 6.0, 10.0),     # Wrist roll
        (-3.14, 3.14, 3.0, 8.0, 8.0),    # Hand
    ]

    for min_pos, max_pos, max_vel, max_acc, max_effort in joint_configs:
        joint_limits.append(JointLimits(
            min_position=min_pos,
            max_position=max_pos,
            min_velocity=-max_vel,
            max_velocity=max_vel,
            min_acceleration=-max_acc,
            max_acceleration=max_acc,
            max_effort=max_effort
        ))

    workspace = WorkspaceBoundary(
        center=(0.5, 0.0, 0.5),
        radius=0.8,
        min_z=0.0,
        max_z=1.5
    )

    return SafetyConfig(
        joint_limits=joint_limits,
        workspace=workspace,
        human_safety_distance=0.5
    )


def main():
    """Demonstration of safety monitoring."""
    # Create safety configuration
    config = create_default_config()

    # Create safety monitor
    safety_monitor = SafetyMonitor(config)

    # Setup callbacks
    def on_violation(v: SafetyViolation):
        print(f"VIOLATION [{v.safety_level.name}]: {v.message}")

    def on_level_change(level: SafetyLevel):
        print(f"SAFETY LEVEL CHANGED: {level.name}")

    safety_monitor.on_violation = on_violation
    safety_monitor.on_safety_level_change = on_level_change

    # Start monitoring
    safety_monitor.start()

    # Test joint limit checking
    print("\n=== Testing Joint Limits ===")
    test_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Normal
    level, violations = safety_monitor.check_all(
        test_positions,
        [0.0] * 7,
        (0.5, 0.0, 0.5)
    )
    print(f"Normal positions - Safety level: {level.name}, Violations: {len(violations)}")

    # Test position near limit
    test_positions = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Near limit
    level, violations = safety_monitor.check_all(
        test_positions,
        [0.0] * 7,
        (0.5, 0.0, 0.5)
    )
    print(f"Near limit - Safety level: {level.name}, Violations: {len(violations)}")

    # Test position over limit
    test_positions = [3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Over limit
    level, violations = safety_monitor.check_all(
        test_positions,
        [0.0] * 7,
        (0.5, 0.0, 0.5)
    )
    print(f"Over limit - Safety level: {level.name}, Violations: {len(violations)}")

    # Test workspace boundary
    print("\n=== Testing Workspace Boundary ===")
    level, violations = safety_monitor.check_all(
        [0.0] * 7,
        [0.0] * 7,
        (2.0, 0.0, 0.5)  # Outside workspace
    )
    print(f"Outside workspace - Safety level: {level.name}, Violations: {len(violations)}")

    # Test human proximity
    print("\n=== Testing Human Proximity ===")
    level, violations = safety_monitor.check_all(
        [0.0] * 7,
        [0.0] * 7,
        (0.5, 0.0, 0.5),
        human_positions=[(0.55, 0.0, 0.5)]  # Human very close
    )
    print(f"Human close - Safety level: {level.name}, Violations: {len(violations)}")

    # Test velocity checking
    print("\n=== Testing Velocities ===")
    level, violations = safety_monitor.check_velocities(
        [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # High velocity
        time.time()
    )
    print(f"High velocity - Safety level: {level.name}, Violations: {len(violations)}")

    # Test emergency stop
    print("\n=== Testing Emergency Stop ===")
    safety_monitor.trigger_e_stop("Test emergency stop")
    status = safety_monitor.get_status()
    print(f"After E-stop - Status: {status}")

    # Reset and verify
    safety_monitor.reset_e_stop("Test complete")
    status = safety_monitor.get_status()
    print(f"After reset - Status: {status}")

    # Stop monitoring
    safety_monitor.stop()

    print("\n=== Summary ===")
    print(f"Total violations recorded: {len(safety_monitor.violation_history)}")
    print(f"Active violations: {len(safety_monitor.active_violations)}")


if __name__ == '__main__':
    main()
>>>>>>> 5cf5f6818272e652910633754d301908ff0bb236
