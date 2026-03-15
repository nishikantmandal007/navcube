---
layout: default
title: Getting Started
sort: 2
---

# Getting Started

This guide walks you through installing NavCube, understanding how it's put together, and wiring it up to your renderer for the first time.

---

## Prerequisites

| Requirement | Version |
|:------------|:--------|
| Python | 3.10 or later |
| PySide6 | Any recent release (6.4+) |
| NumPy | Any recent release (1.23+) |

If you're planning to use one of the built-in connectors, you'll also need:

| Package | What it's for |
|:--------|:--------------|
| pythonocc-core | `OCCNavCubeSync` connector |
| vtk | `VTKNavCubeSync` connector |

---

## Installation

```bash
# Core only — no renderer dependencies
pip install navcube

# With OCC support
pip install navcube[occ]

# With VTK support
pip install navcube[vtk]
```

To confirm everything installed correctly:

```python
from navcube import NavCubeOverlay, NavCubeStyle
print("navcube installed successfully")
```

---

## How it's structured

NavCube keeps your 3D renderer completely separate from the widget itself. The two sides talk through a pair of hooks:

```
┌─────────────────────┐          ┌──────────────────────┐
│   Your 3D Renderer  │          │   NavCubeOverlay     │
│   (OCC / VTK / …)   │          │   (PySide6 QWidget)  │
│                      │          │                      │
│  Camera changes ─────┼── push_camera() ──►  Redraws   │
│                      │          │          cube        │
│  SetProj / SetUp ◄───┼── viewOrientationRequested ──  │
│                      │          │  (user clicked face) │
└─────────────────────┘          └──────────────────────┘
```

The widget never imports or links against any 3D library. All it needs from you is:

1. **`push_camera(dx, dy, dz, ux, uy, uz)`** — call this to tell the cube where your camera is pointing.
2. **`viewOrientationRequested` signal** — connect this to update your renderer when the user clicks a face, edge, corner, or arrow button.

---

## Basic integration

This pattern works with any 3D engine:

```python
from navcube import NavCubeOverlay

# 1. Create the overlay, parented to your viewport widget
cube = NavCubeOverlay(parent=your_3d_widget)
cube.show()

# 2. Connect the signal to update your renderer's camera
def on_orientation(dx, dy, dz, ux, uy, uz):
    """
    dx/dy/dz = OUTWARD camera direction (eye position relative to scene center).
    ux/uy/uz = Camera up vector.
    """
    your_renderer.set_camera_direction(dx, dy, dz)
    your_renderer.set_camera_up(ux, uy, uz)
    your_renderer.redraw()

cube.viewOrientationRequested.connect(on_orientation)

# 3. Push camera state whenever it changes (e.g. in your render loop or mouse handler)
def on_camera_changed():
    d = your_renderer.get_camera_direction()  # INWARD: eye -> scene
    u = your_renderer.get_camera_up()
    cube.push_camera(d.x, d.y, d.z, u.x, u.y, u.z)
```

---

## Understanding the signals

### `viewOrientationRequested(dx, dy, dz, ux, uy, uz)`

Emitted whenever the user interacts with the cube — clicking a face, edge, corner, orbit button, roll arrow, home button, or the backside dot. The six floats are:

- `dx, dy, dz` — the **outward** camera direction (from scene center toward the eye). This is ready for OCC `SetProj()` or an equivalent call.
- `ux, uy, uz` — the camera **up** vector.

During a face-click animation (default 240 ms), the signal fires on every animation tick so your renderer follows the smooth transition in real time.

### `push_camera(dx, dy, dz, ux, uy, uz)`

Call this to sync the cube with your renderer's current camera:

- `dx, dy, dz` — the **inward** camera direction (eye toward scene). For OCC, this is `cam.Direction()` directly.
- `ux, uy, uz` — the camera **up** vector.

**The sign convention matters:** `push_camera` takes the **inward** direction; `viewOrientationRequested` emits the **outward** direction. NavCube handles the negation internally, so you never need to flip signs yourself.

---

## SLERP smoothing during interaction

When the user is actively dragging the camera, call `set_interaction_active()` to tell the cube:

```python
# Wire this into your viewport's mouse handlers:
def mousePressEvent(self, event):
    cube.set_interaction_active(True)
    # ... your orbit logic ...

def mouseReleaseEvent(self, event):
    cube.set_interaction_active(False)
    # ... your orbit logic ...
```

While active, `push_camera()` applies quaternion SLERP smoothing (alpha = 0.65) to absorb momentary renderer instabilities — things like OCC's up-vector flicker near gimbal-lock poles. This keeps the cube visually stable during fast orbiting while adding only ~8 ms of effective visual lag. When interaction ends, smoothing turns off and camera state is applied directly.

---

## Overlay vs inline mode

The `overlay` constructor parameter changes how the widget behaves:

```python
# Overlay mode (default): transparent floating window
cube = NavCubeOverlay(parent=viewport, overlay=True)

# Inline mode: a regular QWidget you can put in any layout
cube = NavCubeOverlay(parent=None, overlay=False)
layout.addWidget(cube)
```

| Feature | Overlay (`True`) | Inline (`False`) |
|:--------|:-----------------|:-----------------|
| Window flags | `Tool + FramelessWindowHint + NoDropShadowWindowHint` | Standard QWidget |
| Background | Translucent | Opaque (respects parent palette) |
| Positioning | Manual (`move()` / `raise_()`) | Managed by layout |
| Best for | Floating over a 3D viewport | Sidebar, dock, or toolbar |

---

## Complete working example

Here's a standalone window with a NavCube that prints orientation changes to the console — good for confirming everything is wired correctly:

```python
import sys
import math
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import Qt, QTimer
from navcube import NavCubeOverlay, NavCubeStyle

class DemoWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NaviCube Demo")
        self.resize(600, 500)

        layout = QVBoxLayout(self)
        self.label = QLabel("Click a cube face to change orientation")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)

        # Create the navicube as an overlay on this window
        self.cube = NavCubeOverlay(parent=self)
        self.cube.viewOrientationRequested.connect(self.on_orient)
        self.cube.show()

        self._reposition_cube()

    def on_orient(self, dx, dy, dz, ux, uy, uz):
        self.label.setText(
            f"Direction: ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})\n"
            f"Up:        ({ux:+.3f}, {uy:+.3f}, {uz:+.3f})"
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_cube()

    def _reposition_cube(self):
        cw = self.cube.width()
        self.cube.move(self.width() - cw - 10, 10)
        self.cube.raise_()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = DemoWindow()
    win.show()
    sys.exit(app.exec())
```
