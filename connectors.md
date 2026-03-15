---
layout: default
title: Connectors
sort: 4
---

# Connectors

A connector is a small bridge between a specific 3D renderer and `NavCubeOverlay`. It handles camera polling, signal wiring, interaction state management, and teardown — all the plumbing you'd otherwise have to write yourself.

NavCube ships with connectors for OCC and VTK. If you're using something else, writing your own takes about 50 lines.

---

## OCC connector (`OCCNavCubeSync`)

Lives in `navcube.connectors.occ`. It's imported lazily, so the base `navcube` package never pulls in OCC unless you ask for it.

### Install

```bash
pip install navcube[occ]
```

### What it does

1. Polls the OCC `V3d_View` camera on a QTimer (default 16 ms tick)
2. Calls `navicube.push_camera()` with the camera's inward direction and up vector
3. Connects `viewOrientationRequested` to update the OCC camera atomically (SetEye + SetUp + single Redraw)
4. Forwards `set_interaction_active()` to the cube for SLERP smoothing
5. Adjusts the poll rate: every tick during interaction (~16 ms), every 4 ticks at idle (~64 ms)

### Full example

```python
from navcube import NavCubeOverlay
from navcube.connectors.occ import OCCNavCubeSync

# 1. Create the navicube overlay, parented to your OCC viewport widget
navicube = NavCubeOverlay(parent=viewport_widget)
navicube.show()

# 2. Create the connector — polling starts immediately
sync = OCCNavCubeSync(occ_view, navicube)

# 3. Wire interaction state in your viewer's mouse handlers
class MyViewer(QWidget):
    def mousePressEvent(self, event):
        sync.set_interaction_active(True)
        # ... your OCC orbit/pan logic ...

    def mouseReleaseEvent(self, event):
        sync.set_interaction_active(False)
        # ... your OCC orbit/pan logic ...

# 4. Clean up when the viewer is destroyed
def on_viewer_closing():
    sync.teardown()
```

### Why `SetEye` instead of `SetProj`

The connector uses `Camera.SetEye()` + `Camera.SetUp()` rather than `V3d_View.SetProj()` + `V3d_View.SetUp()`. This avoids OCC's internal `ImmediateUpdate()` triggering an intermediate render with the new direction but the old up-vector — which was the source of per-frame flicker during animation.

A fallback to `SetProj` / `SetUp` is included for older pythonocc builds that don't have the Camera handle API.

### Poll rate constants

| Constant | Value | Description |
|:---------|:------|:------------|
| `_TICK_MS` | `16` | Base timer interval in ms |
| `_INTERACTION_TICKS` | `1` | Poll every tick during interaction (~16 ms lag) |
| `_IDLE_TICKS` | `4` | Poll every 4 ticks when idle (~64 ms lag) |

---

## VTK connector (`VTKNavCubeSync`)

Same structure as the OCC connector, just bridging a VTK renderer instead.

### Install

```bash
pip install navcube[vtk]
```

### Full example

```python
from navcube import NavCubeOverlay
from navcube.connectors.vtk import VTKNavCubeSync

navicube = NavCubeOverlay(parent=vtk_widget)
navicube.show()

sync = VTKNavCubeSync(vtk_renderer, navicube)

# In your interactor style's mouse handlers:
# sync.set_interaction_active(True)   # on press
# sync.set_interaction_active(False)  # on release

# When you're done:
# sync.teardown()
```

### VTK camera mapping

VTK uses a slightly different camera convention:

- `camera.GetDirectionOfProjection()` returns the inward direction (eye → focal point) — maps directly to `push_camera()`.
- `camera.GetViewUp()` returns the up vector.
- For `viewOrientationRequested`, the outward direction is used to compute the new eye position relative to the focal point.

---

## Writing your own connector

If your engine isn't OCC or VTK, a custom connector takes about 50 lines. You need to handle four things:

1. **Poll the camera** on a timer and call `push_camera()`
2. **Connect the signal** `viewOrientationRequested` to update your renderer
3. **Forward interaction state** via `set_interaction_active()`
4. **Tear down** cleanly when the view is destroyed

### Step 1 — Poll the camera

Set up a QTimer to read your camera state and push it to the cube:

```python
import math
import numpy as np
from PySide6.QtCore import QTimer

class MyEngineNaviCubeSync:
    _TICK_MS = 16
    _INTERACTION_TICKS = 1
    _IDLE_TICKS = 4

    def __init__(self, renderer, navicube):
        self._renderer = renderer
        self._navicube = navicube
        self._interaction_active = False
        self._tick_count = 0

        # Connect the signal (Step 2)
        navicube.viewOrientationRequested.connect(self._on_orientation)

        # Start polling
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(self._TICK_MS)
```

### Step 2 — Handle orientation requests

When the user clicks a face, update your renderer's camera:

```python
    def _on_orientation(self, dx, dy, dz, ux, uy, uz):
        """
        dx/dy/dz = OUTWARD direction (scene -> eye)
        ux/uy/uz = up vector
        """
        if self._renderer is None:
            return

        focal = self._renderer.get_focal_point()
        dist = self._renderer.get_camera_distance()

        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag < 1e-6:
            return
        scale = dist / mag

        self._renderer.set_camera_position(
            focal[0] + dx * scale,
            focal[1] + dy * scale,
            focal[2] + dz * scale,
        )
        self._renderer.set_camera_up(ux, uy, uz)
        self._renderer.redraw()
```

### Step 3 — Manage interaction state

Forward the active/inactive state and adjust polling frequency:

```python
    def set_interaction_active(self, active: bool):
        self._interaction_active = bool(active)
        self._tick_count = 0  # Force immediate read on state change
        if self._navicube is not None:
            self._navicube.set_interaction_active(active)
```

### Step 4 — Implement the poll tick

Read the camera at the right rate and push it to the cube:

```python
    def _tick(self):
        if self._renderer is None or self._navicube is None:
            return

        poll_every = (
            self._INTERACTION_TICKS
            if self._interaction_active
            else self._IDLE_TICKS
        )
        self._tick_count += 1
        if self._tick_count < poll_every:
            return
        self._tick_count = 0

        try:
            # Read INWARD direction (eye -> scene)
            d = self._renderer.get_camera_direction()
            u = self._renderer.get_camera_up()
            self._navicube.push_camera(
                d[0], d[1], d[2],
                u[0], u[1], u[2],
            )
        except Exception:
            pass  # Renderer not ready or being torn down
```

### Step 5 — Teardown

Stop the timer and disconnect signals:

```python
    def teardown(self):
        self._timer.stop()
        try:
            if self._navicube is not None:
                self._navicube.viewOrientationRequested.disconnect(
                    self._on_orientation
                )
        except Exception:
            pass
        self._renderer = None
        self._navicube = None
```

### Complete connector skeleton

Here's the whole thing together — copy and adapt for your engine:

```python
"""
my_engine_sync.py -- NavCubeOverlay connector for MyEngine
"""
import math
from PySide6.QtCore import QTimer


class MyEngineNaviCubeSync:
    _TICK_MS           = 16
    _INTERACTION_TICKS = 1
    _IDLE_TICKS        = 4

    def __init__(self, renderer, navicube):
        self._renderer = renderer
        self._navicube = navicube
        self._interaction_active = False
        self._tick_count = 0

        navicube.viewOrientationRequested.connect(self._on_orientation)
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(self._TICK_MS)

    def set_interaction_active(self, active: bool):
        self._interaction_active = bool(active)
        self._tick_count = 0
        if self._navicube is not None:
            self._navicube.set_interaction_active(active)

    def teardown(self):
        self._timer.stop()
        try:
            if self._navicube is not None:
                self._navicube.viewOrientationRequested.disconnect(
                    self._on_orientation
                )
        except Exception:
            pass
        self._renderer = None
        self._navicube = None

    def _tick(self):
        if self._renderer is None or self._navicube is None:
            return
        poll_every = (
            self._INTERACTION_TICKS
            if self._interaction_active
            else self._IDLE_TICKS
        )
        self._tick_count += 1
        if self._tick_count < poll_every:
            return
        self._tick_count = 0

        try:
            d = self._renderer.get_camera_direction()  # INWARD
            u = self._renderer.get_camera_up()
            self._navicube.push_camera(d[0], d[1], d[2], u[0], u[1], u[2])
        except Exception:
            pass

    def _on_orientation(self, dx, dy, dz, ux, uy, uz):
        if self._renderer is None:
            return
        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag < 1e-6:
            return
        try:
            focal = self._renderer.get_focal_point()
            dist = self._renderer.get_camera_distance()
            scale = dist / mag
            self._renderer.set_camera_position(
                focal[0] + dx * scale,
                focal[1] + dy * scale,
                focal[2] + dz * scale,
            )
            self._renderer.set_camera_up(ux, uy, uz)
            self._renderer.redraw()
        except Exception:
            pass
```
