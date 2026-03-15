---
layout: default
title: Examples
sort: 3
---

# Examples

Ready-to-run code for the most common scenarios. Copy, paste, and adapt.

---

## Minimal overlay

The simplest possible integration — a NavCube floating in the corner of a window:

```python
import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from navcube import NavCubeOverlay

app = QApplication(sys.argv)
win = QMainWindow()
win.setWindowTitle("Minimal NaviCube")
win.resize(800, 600)
win.show()

cube = NavCubeOverlay(parent=win)
cube.move(win.width() - 150, 10)
cube.show()

cube.viewOrientationRequested.connect(
    lambda dx, dy, dz, ux, uy, uz:
        print(f"Dir: ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})  "
              f"Up: ({ux:+.3f}, {uy:+.3f}, {uz:+.3f})")
)

sys.exit(app.exec())
```

---

## Inline / dock mode

Embed the cube as a regular widget inside a layout — no overlay window flags:

```python
import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout,
    QVBoxLayout, QLabel, QDockWidget,
)
from PySide6.QtCore import Qt
from navcube import NavCubeOverlay, NavCubeStyle

app = QApplication(sys.argv)
win = QMainWindow()
win.setWindowTitle("Inline NaviCube Demo")
win.resize(900, 600)

# Main content area
central = QLabel("3D Viewport Area")
central.setAlignment(Qt.AlignCenter)
win.setCentralWidget(central)

# Dock widget with an inline navicube
dock = QDockWidget("Orientation", win)
dock_widget = QWidget()
dock_layout = QVBoxLayout(dock_widget)

# overlay=False makes it a normal QWidget
cube = NavCubeOverlay(parent=None, overlay=False,
                       style=NavCubeStyle(size=100, inactive_opacity=1.0))
dock_layout.addWidget(cube)
dock_layout.addStretch()

dock.setWidget(dock_widget)
win.addDockWidget(Qt.RightDockWidgetArea, dock)

cube.viewOrientationRequested.connect(
    lambda dx, dy, dz, ux, uy, uz:
        central.setText(f"Dir: ({dx:+.2f}, {dy:+.2f}, {dz:+.2f})")
)

win.show()
sys.exit(app.exec())
```

---

## Custom colors

A dark blue theme with orange hover highlights:

```python
import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from navcube import NavCubeOverlay, NavCubeStyle

app = QApplication(sys.argv)
win = QMainWindow()
win.resize(800, 600)
win.show()

style = NavCubeStyle(
    theme="dark",
    size=140,

    # Dark blue faces
    face_color_dark=(35, 55, 110),
    edge_color_dark=(25, 40, 85),
    corner_color_dark=(18, 28, 60),

    # Light text
    text_color_dark=(220, 230, 255),

    # Dark borders
    border_color_dark=(8, 12, 30),
    border_secondary_color_dark=(12, 18, 40),

    # Orange hover highlight
    hover_color_dark=(255, 140, 0, 235),
    hover_text_color_dark=(255, 255, 255),

    # Subtle semi-transparent controls
    control_color_dark=(25, 45, 90, 100),
    control_rim_color_dark=(15, 30, 60, 150),

    # Stronger shadow
    shadow_color_dark=(0, 0, 0, 100),
    shadow_offset_x=2.5,
    shadow_offset_y=3.0,
)

cube = NavCubeOverlay(parent=win, style=style)
cube.move(win.width() - 170, 10)
cube.show()

win.show()
sys.exit(app.exec())
```

---

## Localized labels

### German

```python
from navcube import NavCubeStyle

german_style = NavCubeStyle(
    labels={
        "TOP": "OBEN",
        "FRONT": "VORNE",
        "LEFT": "LINKS",
        "BACK": "HINTEN",
        "RIGHT": "RECHTS",
        "BOTTOM": "UNTEN",
    },
)
```

### Japanese

```python
from navcube import NavCubeStyle

japanese_style = NavCubeStyle(
    font_family="Noto Sans CJK JP",
    font_fallback="SansSerif",
    font_weight="bold",
    label_max_width_ratio=0.60,   # CJK characters need more width
    label_max_height_ratio=0.55,
    labels={
        "TOP": "\u4e0a",      # 上
        "FRONT": "\u524d",    # 前
        "LEFT": "\u5de6",     # 左
        "BACK": "\u5f8c",     # 後
        "RIGHT": "\u53f3",    # 右
        "BOTTOM": "\u4e0b",   # 下
    },
)
```

### Russian

```python
from navcube import NavCubeStyle

russian_style = NavCubeStyle(
    font_family="DejaVu Sans",
    labels={
        "TOP": "\u0412\u0415\u0420\u0425",         # ВЕРХ
        "FRONT": "\u041f\u0415\u0420\u0415\u0414",  # ПЕРЕД
        "LEFT": "\u041b\u0415\u0412\u041e",         # ЛЕВО
        "BACK": "\u0417\u0410\u0414",               # ЗАД
        "RIGHT": "\u041f\u0420\u0410\u0412\u041e",  # ПРАВО
        "BOTTOM": "\u041d\u0418\u0417",             # НИЗ
    },
)
```

---

## Y-up engine

For engines that use Y-up coordinates (Three.js, Unity, glTF viewers):

```python
import sys
import numpy as np
from PySide6.QtWidgets import QApplication, QMainWindow
from navcube import NavCubeOverlay

class YUpNaviCube(NavCubeOverlay):
    """NaviCube configured for Y-up coordinate systems."""
    _WORLD_ROT = np.array([
        [1,  0,  0],
        [0,  0, -1],
        [0,  1,  0],
    ], dtype=float)

app = QApplication(sys.argv)
win = QMainWindow()
win.resize(800, 600)
win.show()

cube = YUpNaviCube(parent=win)
cube.move(win.width() - 150, 10)
cube.show()

def on_orient(dx, dy, dz, ux, uy, uz):
    # These are in Y-up world space — no conversion needed
    print(f"Y-up Dir: ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})")
    print(f"Y-up Up:  ({ux:+.3f}, {uy:+.3f}, {uz:+.3f})")

cube.viewOrientationRequested.connect(on_orient)

# Push Y-up camera vectors directly
# Example: looking from front, direction = (0, 0, -1), up = (0, 1, 0)
cube.push_camera(0, 0, -1, 0, 1, 0)

sys.exit(app.exec())
```

---

## OCC integration

Full working integration with pythonocc:

```python
import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import Qt

from OCC.Display.SimpleGui import init_display
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox

from navcube import NavCubeOverlay
from navcube.connectors.occ import OCCNavCubeSync


class OCCViewerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OCC + NaviCube")
        self.resize(1024, 768)

        # Initialize OCC display (replace with your own OCC widget setup)
        display, start_display, add_menu, add_function_to_menu = init_display()
        self._display = display

        # Add some geometry
        box = BRepPrimAPI_MakeBox(50, 30, 20).Shape()
        display.DisplayShape(box, update=True)

        # Get the V3d_View
        view = display.GetView()

        # Create and connect the navicube
        canvas = display.GetWidget()
        self.navicube = NavCubeOverlay(parent=canvas)
        self.navicube.show()

        # The connector handles everything else
        self.sync = OCCNavCubeSync(view, self.navicube)

        self.navicube.move(canvas.width() - 150, 10)

    def closeEvent(self, event):
        self.sync.teardown()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = OCCViewerWindow()
    win.show()
    sys.exit(app.exec())
```

---

## VTK integration

Full working integration with VTK:

```python
import sys
from PySide6.QtWidgets import QApplication, QMainWindow
import vtkmodules.all as vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from navcube import NavCubeOverlay
from navcube.connectors.vtk import VTKNavCubeSync


class VTKViewerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VTK + NaviCube")
        self.resize(1024, 768)

        # VTK setup
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.setCentralWidget(self.vtk_widget)

        renderer = vtk.vtkRenderer()
        renderer.SetBackground(0.2, 0.2, 0.2)
        self.vtk_widget.GetRenderWindow().AddRenderer(renderer)

        # Add a cube actor
        source = vtk.vtkCubeSource()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        renderer.AddActor(actor)
        renderer.ResetCamera()

        # Create the navicube
        self.navicube = NavCubeOverlay(parent=self.vtk_widget)
        self.navicube.show()
        self.navicube.move(self.vtk_widget.width() - 150, 10)

        # Connect via VTK sync
        self.sync = VTKNavCubeSync(renderer, self.navicube)

        self.vtk_widget.Initialize()
        self.vtk_widget.Start()

    def closeEvent(self, event):
        self.sync.teardown()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = VTKViewerWindow()
    win.show()
    sys.exit(app.exec())
```

---

## Runtime theme switching

Toggle between light and dark themes with a button:

```python
import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget,
)
from navcube import NavCubeOverlay, NavCubeStyle

app = QApplication(sys.argv)
win = QMainWindow()
win.setWindowTitle("Theme Switching")
win.resize(800, 600)

central = QWidget()
layout = QVBoxLayout(central)
win.setCentralWidget(central)

cube = NavCubeOverlay(parent=win)
cube.show()
cube.move(win.width() - 150, 10)

is_dark = [False]

light_style = NavCubeStyle(theme="light")
dark_style = NavCubeStyle(
    theme="dark",
    face_color_dark=(60, 60, 70),
    edge_color_dark=(45, 45, 55),
    corner_color_dark=(35, 35, 42),
    hover_color_dark=(0, 180, 100, 235),
)

def toggle_theme():
    is_dark[0] = not is_dark[0]
    cube.set_style(dark_style if is_dark[0] else light_style)
    btn.setText("Switch to Light" if is_dark[0] else "Switch to Dark")

btn = QPushButton("Switch to Dark")
btn.clicked.connect(toggle_theme)
layout.addWidget(btn)
layout.addStretch()

win.show()
sys.exit(app.exec())
```

---

## Custom home orientation

Set the orientation the Home button snaps back to:

```python
import math
from navcube import NavCubeOverlay

cube = NavCubeOverlay(parent=viewport)
cube.show()

# Home = looking straight at the FRONT face
# Inward direction: (0, 1, 0) — toward +Y (FRONT in Z-up)
# Up vector: (0, 0, 1) — Z is up
cube.set_home(0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

# Or set home to an isometric view
d = 1.0 / math.sqrt(3.0)
cube.set_home(-d, d, -d, 0.0, 0.0, 1.0)
```

---

## XYZ gizmo

Show the axis gizmo with custom colors:

```python
from navcube import NavCubeOverlay, NavCubeStyle

style = NavCubeStyle(
    show_gizmo=True,
    gizmo_x_color=(255, 50, 50),    # Bright red
    gizmo_y_color=(50, 255, 50),    # Bright green
    gizmo_z_color=(50, 100, 255),   # Bright blue
    gizmo_font_size=10,
)

cube = NavCubeOverlay(parent=viewport, style=style)
cube.show()
```

---

## Custom connector skeleton

A minimal template for writing your own connector. Replace the placeholder API calls with whatever your engine provides:

```python
"""
Connector template for NavCubeOverlay with a custom 3D engine.
Replace the placeholder method calls with your engine's API.
"""
import math
from PySide6.QtCore import QTimer


class CustomNaviCubeSync:
    _TICK_MS = 16
    _INTERACTION_TICKS = 1
    _IDLE_TICKS = 4

    def __init__(self, engine, navicube):
        self._engine = engine
        self._navicube = navicube
        self._active = False
        self._tick_count = 0

        navicube.viewOrientationRequested.connect(self._on_orient)
        self._timer = QTimer()
        self._timer.timeout.connect(self._poll)
        self._timer.start(self._TICK_MS)

    def set_interaction_active(self, active: bool):
        self._active = bool(active)
        self._tick_count = 0
        if self._navicube:
            self._navicube.set_interaction_active(active)

    def teardown(self):
        self._timer.stop()
        try:
            if self._navicube:
                self._navicube.viewOrientationRequested.disconnect(self._on_orient)
        except Exception:
            pass
        self._engine = None
        self._navicube = None

    def _poll(self):
        if not self._engine or not self._navicube:
            return
        rate = self._INTERACTION_TICKS if self._active else self._IDLE_TICKS
        self._tick_count += 1
        if self._tick_count < rate:
            return
        self._tick_count = 0
        try:
            # Replace with your engine's camera API:
            cam_dir = self._engine.get_camera_direction()  # INWARD (eye -> scene)
            cam_up = self._engine.get_camera_up()
            self._navicube.push_camera(
                cam_dir[0], cam_dir[1], cam_dir[2],
                cam_up[0], cam_up[1], cam_up[2],
            )
        except Exception:
            pass

    def _on_orient(self, dx, dy, dz, ux, uy, uz):
        if not self._engine:
            return
        mag = math.sqrt(dx*dx + dy*dy + dz*dz)
        if mag < 1e-6:
            return
        try:
            # Replace with your engine's camera API:
            focal = self._engine.get_focal_point()
            dist = self._engine.get_camera_distance()
            s = dist / mag
            self._engine.set_camera_position(
                focal[0] + dx*s, focal[1] + dy*s, focal[2] + dz*s,
            )
            self._engine.set_camera_up(ux, uy, uz)
            self._engine.render()
        except Exception:
            pass
```
