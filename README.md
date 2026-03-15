# pyside-navicube

A NaviCube orientation widget for PySide6 applications, inspired by the one in FreeCAD. Drop it into any 3D viewport — it doesn't care whether you're using OpenCASCADE, VTK, raw OpenGL, or something else entirely.

**[Documentation](https://nishikantmandal007.github.io/pyside-navicube/)** | **[PyPI](https://pypi.org/project/pyside-navicube/)** | **[GitHub](https://github.com/nishikantmandal007/pyside-navicube)**

---

## Why this exists

Most NaviCube implementations are tightly coupled to a specific renderer. If you're using pythonocc and want a cube, you're stuck with OCC's built-in one (which is hard to style and lives inside the OpenGL context). Same story with VTK.

This library keeps the widget completely separate from any renderer. The cube is just a PySide6 widget that emits signals — you decide what happens when someone clicks a face. There's a ready-made connector for OCC and VTK, but wiring up anything else takes about 10 lines.

---

## Install

```bash
pip install pyside-navicube
```

With the OCC connector:
```bash
pip install pyside-navicube[occ]
```

With VTK:
```bash
pip install pyside-navicube[vtk]
```

---

## Basic usage

```python
from navicube import NaviCubeOverlay

cube = NaviCubeOverlay(parent=your_window)
cube.show()

# Tell the navicube where the camera is pointing
# dx/dy/dz = inward direction (eye → scene), ux/uy/uz = up vector
cube.push_camera(dx, dy, dz, ux, uy, uz)

# React when someone clicks a face
cube.viewOrientationRequested.connect(your_camera_update_fn)
# px/py/pz comes out as outward direction — pass straight to OCC SetProj

# Let the cube know when a drag starts/ends (enables smoothing)
cube.set_interaction_active(True)   # on mouse press
cube.set_interaction_active(False)  # on mouse release
```

---

## OCC / pythonocc

If you're using pythonocc, the `OCCNaviCubeSync` connector handles all of the above automatically — polling, signal wiring, interaction hints, the works.

```python
from navicube import NaviCubeOverlay
from navicube.connectors.occ import OCCNaviCubeSync

# Parent to your tab or window, NOT to the OCC canvas itself
# (avoids OpenGL repaint corruption on Linux)
cube = NaviCubeOverlay(parent=tab_widget)
cube.show()

# Call this once your OCC view is initialised
sync = OCCNaviCubeSync(occ_view, cube)

# In mousePressEvent / mouseReleaseEvent on your viewer:
sync.set_interaction_active(True)
sync.set_interaction_active(False)

# Clean up when the view is torn down
sync.teardown()
```

---

## VTK / PyVista

```python
from navicube import NaviCubeOverlay
from navicube.connectors.vtk import VTKNaviCubeSync

cube = NaviCubeOverlay(parent=vtk_widget)
cube.show()

sync = VTKNaviCubeSync(renderer, cube, render_window=vtk_widget.GetRenderWindow())
```

---

## Styling

Everything visual is configurable through `NaviCubeStyle`. You can set face colors, hover colors, font, animation speed, opacity — all of it. Changes can be applied at runtime too.

```python
from navicube import NaviCubeOverlay, NaviCubeStyle

style = NaviCubeStyle(
    size=140,
    face_color=(215, 220, 245),
    edge_color=(242, 198, 208),
    corner_color=(195, 228, 210),
    hover_color=(218, 62, 112, 250),
    text_color=(28, 26, 68),
    show_gizmo=True,
    inactive_opacity=0.65,
    animation_ms=300,
)

cube = NaviCubeOverlay(parent=your_window, style=style)

# Or change it later
cube.set_style(NaviCubeStyle(size=160, theme="dark"))
```

The full list of style options is in the [Style Reference](https://nishikantmandal007.github.io/pyside-navicube/style-reference).

---

## Coordinate conventions

The cube uses **Z-up** by default (same as OCC, FreeCAD, Blender).

| Direction | Convention |
|---|---|
| `push_camera` dx/dy/dz | Inward (eye → scene) — same as OCC `cam.Direction()` |
| `viewOrientationRequested` px/py/pz | Outward (scene → eye) — ready for OCC `SetProj` |

For Y-up engines (Three.js, Unity, GLTF):

```python
import numpy as np
from navicube import NaviCubeOverlay

class YUpNaviCube(NaviCubeOverlay):
    _WORLD_ROT = np.array([
        [1, 0,  0],
        [0, 0, -1],
        [0, 1,  0],
    ], dtype=float)
```

---

## Inline / dock mode

By default the cube is an overlay (floats over the viewport). To use it as a regular widget inside a layout:

```python
cube = NaviCubeOverlay(parent=panel, overlay=False)
layout.addWidget(cube)
```

---

## Writing a connector for another renderer

Three things:
1. Poll or subscribe to camera changes → call `cube.push_camera()`
2. Connect `cube.viewOrientationRequested` → update your renderer camera
3. Call `cube.set_interaction_active(True/False)` on drag start/end

`navicube/connectors/occ.py` is a complete reference (~100 lines).

---

## Requirements

- Python 3.10+
- PySide6
- numpy

Optional: `pythonocc-core` (for the OCC connector), `vtk` (for the VTK connector).

---

## License

LGPL-2.1
