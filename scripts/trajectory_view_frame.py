"""Display rotation for a Gazebo camera looking along world yaw (data stays world XY)."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ViewFrame:
    # Clearpath's installed gui.config: camera_pose = -6 0 6 0 0.5 0.
    # With roll=0, camera right is -Y and its horizontal look direction is +X.
    yaw_deg: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.yaw_deg):
            raise ValueError("View yaw must be finite")
        radians = math.radians(self.yaw_deg)
        object.__setattr__(self, "sine", round(math.sin(radians), 15))
        object.__setattr__(self, "cosine", round(math.cos(radians), 15))

    def project(self, x, y):
        """World XY -> screen right/up metres; an orthogonal plan view."""
        return self.sine*x-self.cosine*y, self.cosine*x+self.sine*y

    def unproject(self, horizontal, vertical):
        """Mouse plot coordinates -> world XY, inverse of project()."""
        return (self.sine*horizontal+self.cosine*vertical,
                -self.cosine*horizontal+self.sine*vertical)

    def series(self, points):
        projected = [self.project(x, y) for x, y in points]
        return [p[0] for p in projected], [p[1] for p in projected]

    def coordinate_text(self, horizontal, vertical):
        x, y = self.unproject(horizontal, vertical)
        return f"World X={x:.3f} m, Y={y:.3f} m"

    def decorate(self, axis):
        yaw = (self.yaw_deg+180) % 360-180
        labels = {
            0:("-World Y (m)", "World X (m)"),
            90:("World X (m)", "World Y (m)"),
            -180:("World Y (m)", "-World X (m)"),
            -90:("-World X (m)", "-World Y (m)"),
        }
        horizontal, vertical = labels.get(yaw, ("View horizontal (m)", "View vertical (m)"))
        axis.set_xlabel(horizontal)
        axis.set_ylabel(vertical)
        axis.format_coord = self.coordinate_text
        artists = []
        # Use physical point offsets so arrows remain correct on nonsquare axes.
        for point, label, color in (((1, 0), "+X", "#c62828"), ((0, 1), "+Y", "#228b22")):
            dx, dy = self.project(*point)
            artists.append(axis.annotate(label, xy=(0.88, 0.13), xycoords="axes fraction",
                xytext=(dx*33, dy*33), textcoords="offset points", color=color,
                ha="center", va="center", fontsize=9, fontweight="bold",
                arrowprops={"arrowstyle":"<-", "color":color, "linewidth":1.3}))
        return artists
