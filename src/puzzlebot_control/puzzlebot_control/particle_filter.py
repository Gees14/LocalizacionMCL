"""Monte Carlo Localization — pure particle filter with no ROS dependencies."""

import math
import random
from typing import List, Tuple

from .map_loader import OccupancyMap


class Particle:
    """Single hypothesis about the robot pose in the world frame."""

    __slots__ = ('x', 'y', 'theta', 'weight')

    def __init__(self, x: float, y: float, theta: float, weight: float = 1.0) -> None:
        self.x      = x
        self.y      = y
        self.theta  = theta   # radians, wrapped to (-π, π]
        self.weight = weight

    def __repr__(self) -> str:
        return (
            f"Particle(x={self.x:.3f}, y={self.y:.3f}, "
            f"θ={math.degrees(self.theta):.1f}°, w={self.weight:.4f})"
        )


class ParticleFilter:
    """Monte Carlo Localization for a differential-drive robot.

    The standard MCL loop is split into three explicit steps:

    1. **predict(dx, dy, dtheta)**
       Apply the motion model: move every particle by the odometry delta
       (expressed in the *robot* frame) plus Gaussian noise.

    2. **update(angles, ranges)**
       Apply the sensor model: score each particle by comparing
       ray-cast expected ranges against the laser measurements, then
       resample — keep the top-k survivors and clone them back to n.

    3. **best_estimate() → (x, y, theta)**
       Compute the weighted mean pose of the current particle set.

    Parameters
    ----------
    occ_map        : OccupancyMap  — binary occupancy grid
    num_particles  : total particle count
    survivors      : how many top-weighted particles survive each update
    motion_noise_xy    : std-dev of position noise applied per predict step (m)
    motion_noise_theta : std-dev of heading noise applied per predict step (rad)
    sensor_sigma   : std-dev of the Gaussian beam likelihood (m)
    num_rays       : how many laser rays to sample for scoring
    ray_step       : ray-marching granularity (m)
    max_range      : sensor maximum range (m)
    """

    def __init__(
        self,
        occ_map:            OccupancyMap,
        num_particles:      int   = 500,
        survivors:          int   = 150,
        motion_noise_xy:    float = 0.05,
        motion_noise_theta: float = 0.05,
        sensor_sigma:       float = 0.20,
        num_rays:           int   = 36,
        ray_step:           float = 0.025,
        max_range:          float = 10.0,
    ) -> None:
        self.map          = occ_map
        self.n            = num_particles
        self.k            = survivors
        self.noise_xy     = motion_noise_xy
        self.noise_theta  = motion_noise_theta
        self.sigma        = sensor_sigma
        self.num_rays     = num_rays
        self.ray_step     = ray_step
        self.max_range    = max_range

        self.particles: List[Particle] = []
        self._init_particles()

    # ------------------------------------------------------------------
    # Step 1 — Motion model
    # ------------------------------------------------------------------

    def predict(self, dx: float, dy: float, dtheta: float) -> None:
        """Move every particle by the robot-frame odometry delta plus noise.

        dx, dy are the translation components in the *robot's previous frame*;
        each particle rotates them into its own heading before applying.
        """
        for p in self.particles:
            cos_t = math.cos(p.theta)
            sin_t = math.sin(p.theta)
            p.x     += dx * cos_t - dy * sin_t + random.gauss(0.0, self.noise_xy)
            p.y     += dx * sin_t + dy * cos_t + random.gauss(0.0, self.noise_xy)
            p.theta  = _wrap(p.theta + dtheta + random.gauss(0.0, self.noise_theta))

    # ------------------------------------------------------------------
    # Step 2 — Sensor model + resample
    # ------------------------------------------------------------------

    def update(self, angles: List[float], ranges: List[float]) -> None:
        """Score particles against the laser scan, then resample to n.

        *angles* and *ranges* must already be sub-sampled to ~num_rays entries.
        """
        if not angles:
            return

        for p in self.particles:
            p.weight = self._beam_likelihood(p, angles, ranges)

        # Keep the top-k survivors
        self.particles.sort(key=lambda p: p.weight, reverse=True)
        survivors = self.particles[:self.k]

        # Clone survivors back to n with small perturbation
        self.particles = list(survivors)
        while len(self.particles) < self.n:
            src = random.choice(survivors)
            self.particles.append(Particle(
                x      = src.x     + random.gauss(0.0, self.noise_xy),
                y      = src.y     + random.gauss(0.0, self.noise_xy),
                theta  = _wrap(src.theta + random.gauss(0.0, self.noise_theta)),
                weight = src.weight,
            ))

    # ------------------------------------------------------------------
    # Step 3 — Pose estimate
    # ------------------------------------------------------------------

    def best_estimate(self) -> Tuple[float, float, float]:
        """Weighted mean of all particles.

        Heading uses circular mean to handle the ±π wrap correctly.
        """
        total = sum(p.weight for p in self.particles) or 1.0
        x = sum(p.x * p.weight for p in self.particles) / total
        y = sum(p.y * p.weight for p in self.particles) / total

        sin_sum = sum(math.sin(p.theta) * p.weight for p in self.particles) / total
        cos_sum = sum(math.cos(p.theta) * p.weight for p in self.particles) / total
        theta   = math.atan2(sin_sum, cos_sum)

        return x, y, theta

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init_particles(self) -> None:
        """Seed particles uniformly across free cells in the map."""
        free = self.map.free_cells()
        if not free:
            raise RuntimeError("Map has no free cells — cannot seed particles.")
        for _ in range(self.n):
            wx, wy = random.choice(free)
            self.particles.append(
                Particle(wx, wy, random.uniform(-math.pi, math.pi))
            )

    def _beam_likelihood(
        self,
        p:      Particle,
        angles: List[float],
        ranges: List[float],
    ) -> float:
        """Gaussian beam model: average likelihood over all sampled rays."""
        if not self.map.is_free(p.x, p.y):
            return 0.0

        total_score = 0.0
        for angle, measured in zip(angles, ranges):
            expected = self.map.ray_cast(
                p.x, p.y,
                _wrap(p.theta + angle),
                max_range=self.max_range,
                step=self.ray_step,
            )
            diff         = measured - expected
            total_score += math.exp(-0.5 * (diff / self.sigma) ** 2)

        return total_score / len(angles)


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _wrap(angle: float) -> float:
    """Wrap *angle* into (-π, π]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
