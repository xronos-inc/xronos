# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


class MonteCarloPiAnimation:
    _points_per_frame: int = 5

    def __init__(self, n_frames: int, output_file: str) -> None:
        self.n_frames = n_frames
        self.output_file = output_file
        self.points_inside_circle = 0
        self.points_total = 0

        self.fig, self.ax = plt.subplots()
        self._init_plot()
        self.scatter, self.pi_estimate_text = self._init_scatter_and_text()

    def _init_scatter_and_text(self) -> Tuple[plt.Line2D, plt.Text]:
        """Initialize scatter plot and text for displaying PI estimate."""
        (scatter,) = self.ax.plot([], [], "b.", markersize=3)
        pi_estimate_text = self.ax.text(-0.95, 1.05, "", fontsize=18)
        return scatter, pi_estimate_text

    def _init_plot(self) -> None:
        """Initialize the plot with a circle and equal scaling."""
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        circle = plt.Circle((0, 0), 1, edgecolor="r", facecolor="none")
        self.ax.add_artist(circle)

    def _init_animation(self) -> Tuple[plt.Line2D, plt.Text]:
        """Initialization function for the animation."""
        self.scatter.set_data([], [])
        self.pi_estimate_text.set_text("")
        return self.scatter, self.pi_estimate_text

    def _update_animation(self, frame: int) -> Tuple[plt.Line2D, plt.Text]:
        """Update function for the animation."""
        for _ in range(self._points_per_frame):
            # Generate a random point
            x, y = np.random.uniform(-1, 1, 2)
            self.points_total += 1

            # Check if the point is inside the unit circle
            if x**2 + y**2 <= 1:
                self.points_inside_circle += 1

            # Update the scatter plot
            self.scatter.set_data(
                np.append(self.scatter.get_xdata(), x),
                np.append(self.scatter.get_ydata(), y),
            )

        # Update the PI estimate
        if self.points_total > 0:
            pi_estimate: float = (self.points_inside_circle / self.points_total) * 4
            self.pi_estimate_text.set_text(
                f"\u03c0 \u2248 {pi_estimate:.4f}, n = {self.points_total}"
            )

        return self.scatter, self.pi_estimate_text

    def run(self) -> None:
        """Run the animation and save to file."""
        ani = FuncAnimation(
            self.fig,
            self._update_animation,
            frames=self.n_frames,
            init_func=self._init_animation,
            blit=True,
            repeat=False,
        )
        ani.save(self.output_file, writer="imagemagick", fps=24)


if __name__ == "__main__":
    animation = MonteCarloPiAnimation(500, "monte_carlo_pi.gif")
    animation.run()
