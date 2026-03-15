---
layout: default
title: API Reference
sort: 6
---

# API Reference

Everything public in NavCube — classes, methods, signals, and constants.

---

## `NavCubeOverlay`

```python
from navcube import NavCubeOverlay
```

A PySide6 `QWidget` that renders a FreeCAD-style NaviCube using `QPainter`. It communicates with your 3D renderer exclusively through `push_camera()` (input) and `viewOrientationRequested` (output). No renderer imports, no shared context.

### Constructor

```python
NavCubeOverlay(
    parent: QWidget | None = None,
    *,
    overlay: bool = True,
    style: NavCubeStyle | None = None,
)
```

| Parameter | Type | Default | Description |
|:----------|:-----|:--------|:------------|
| `parent` | `QWidget \| None` | `None` | Qt parent widget. In overlay mode this should be your 3D viewport widget. |
| `overlay` | `bool` | `True` | `True` creates a transparent floating tool window. `False` creates a plain opaque QWidget you can drop into any layout or dock. |
| `style` | `NavCubeStyle \| None` | `None` | Visual and behavioral settings. Defaults to `NavCubeStyle()` if not provided. |

### Signals

#### `viewOrientationRequested`

```python
viewOrientationRequested = Signal(float, float, float, float, float, float)
```

Fires when the user clicks a cube face, edge, corner, or control button. Also fires on every animation tick during the resulting camera transition, so your renderer can follow the smooth movement.

| Parameter | Description |
|:----------|:------------|
| `dx` | Outward camera direction X (scene center → eye) |
| `dy` | Outward camera direction Y |
| `dz` | Outward camera direction Z |
| `ux` | Camera up vector X |
| `uy` | Camera up vector Y |
| `uz` | Camera up vector Z |

The direction is in your application's world space (after `_WORLD_ROT` is applied). It's ready for OCC `SetProj()` or equivalent — no sign flip needed on your end.

### Methods

#### `push_camera`

```python
@Slot(float, float, float, float, float, float)
def push_camera(
    self,
    dx: float, dy: float, dz: float,
    ux: float, uy: float, uz: float,
) -> None
```

Sync the cube with your renderer's current camera state. Call this whenever your camera moves.

| Parameter | Description |
|:----------|:------------|
| `dx` | Inward camera direction X (eye → scene) |
| `dy` | Inward camera direction Y |
| `dz` | Inward camera direction Z |
| `ux` | Camera up vector X |
| `uy` | Camera up vector Y |
| `uz` | Camera up vector Z |

**What happens internally:**
- During a face-click animation, the push is silently ignored (unless interaction is active, in which case the animation cancels).
- During active interaction (`set_interaction_active(True)`), the push is smoothed via quaternion SLERP (alpha 0.65).
- Otherwise, the camera state is applied directly.
- Must be called from the Qt main thread.

**If your render loop runs on a worker thread**, post back using:

```python
QMetaObject.invokeMethod(
    cube, "push_camera", Qt.QueuedConnection,
    Q_ARG(float, dx), Q_ARG(float, dy), Q_ARG(float, dz),
    Q_ARG(float, ux), Q_ARG(float, uy), Q_ARG(float, uz),
)
```

#### `set_home`

```python
def set_home(
    self,
    dx: float, dy: float, dz: float,
    ux: float, uy: float, uz: float,
) -> None
```

Set the orientation that the Home button snaps back to.

| Parameter | Description |
|:----------|:------------|
| `dx` | Inward camera direction X (same convention as `push_camera`) |
| `dy` | Inward camera direction Y |
| `dz` | Inward camera direction Z |
| `ux` | Up vector X |
| `uy` | Up vector Y |
| `uz` | Up vector Z |

The default home is an ISO view expressed in the current `_WORLD_ROT` coordinate system. Override this once after construction (or after loading user preferences) if you want a different starting orientation.

#### `set_interaction_active`

```python
def set_interaction_active(self, active: bool) -> None
```

Tell the cube whether the user is actively dragging the camera.

| Parameter | Description |
|:----------|:------------|
| `active` | `True` on mouse press, `False` on mouse release |

While active, `push_camera()` applies SLERP smoothing (alpha 0.65, ~8 ms effective visual lag) to absorb momentary renderer jitter — particularly OCC's up-vector flicker near gimbal-lock poles.

#### `set_style`

```python
def set_style(self, style: NavCubeStyle) -> None
```

Swap the current style at runtime. Triggers a full rebuild and repaint.

| Parameter | Description |
|:----------|:------------|
| `style` | A `NavCubeStyle` instance |

Internally this: stores the new style, recomputes derived values, clears the font cache, rebuilds face geometry, recalculates DPI scaling and widget size, updates the timer interval, and forces a repaint. Safe to call at any point.

### Class attributes

#### `_WORLD_ROT`

```python
_WORLD_ROT: np.ndarray = np.eye(3)
```

A 3×3 rotation matrix that maps NavCube's internal Z-up space to your application's world space. Override it as a class attribute in a subclass to support Y-up or any other coordinate convention. See [Coordinate Systems](coordinate-systems) for the full explanation.

### Properties

#### `hovered_id`

```python
hovered_id: str | None
```

The ID of the currently hovered element, or `None` if nothing is hovered. Face IDs: `TOP`, `FRONT`, `LEFT`, `BACK`, `RIGHT`, `BOTTOM`, `FRONT_TOP`, `FTR`, etc. Control IDs: `ArrowNorth`, `ArrowSouth`, `ArrowEast`, `ArrowWest`, `ArrowLeft`, `ArrowRight`, `ViewMenu`, `DotBackside`.

---

## `NavCubeStyle`

```python
from navcube import NavCubeStyle
```

A Python `dataclass` with every visual and behavioral setting for the widget. See [Style Reference](style-reference) for exhaustive documentation of each field.

### Constructor

```python
NavCubeStyle(**kwargs)
```

All fields have defaults. Pass only what you want to change:

```python
# All defaults
style = NavCubeStyle()

# Customize a few fields
style = NavCubeStyle(size=150, theme="dark", animation_ms=300)
```

### Fields summary

| Category | Fields |
|:---------|:-------|
| Geometry | `size`, `padding`, `scale`, `chamfer` |
| Animation | `animation_ms`, `tick_ms` |
| Thresholds | `visibility_threshold`, `orbit_step_deg`, `sync_epsilon`, `inactive_opacity` |
| Lighting | `light_direction` |
| Theme | `theme` |
| Light colors | `face_color`, `edge_color`, `corner_color`, `text_color`, `border_color`, `border_secondary_color`, `control_color`, `control_rim_color`, `hover_color`, `hover_text_color`, `dot_color`, `shadow_color` |
| Dark colors | `face_color_dark`, `edge_color_dark`, `corner_color_dark`, `text_color_dark`, `border_color_dark`, `border_secondary_color_dark`, `control_color_dark`, `control_rim_color_dark`, `hover_color_dark`, `hover_text_color_dark`, `dot_color_dark`, `shadow_color_dark` |
| Font | `font_family`, `font_fallback`, `font_weight`, `label_max_width_ratio`, `label_max_height_ratio`, `min_font_size` |
| Labels | `labels` |
| Controls | `show_controls`, `show_gizmo`, `gizmo_x_color`, `gizmo_y_color`, `gizmo_z_color`, `gizmo_font_size` |
| Borders | `border_width_main`, `border_width_secondary`, `control_border_width` |
| Shadow | `shadow_offset_x`, `shadow_offset_y` |

---

## `OCCNavCubeSync`

```python
from navcube.connectors.occ import OCCNavCubeSync
```

Bridges an OCC `V3d_View` with a `NavCubeOverlay`. It handles camera polling, signal wiring, interaction state, and clean teardown — so you don't have to.

### Constructor

```python
OCCNavCubeSync(view, navicube)
```

| Parameter | Type | Description |
|:----------|:-----|:------------|
| `view` | `V3d_View` | An initialized OCC V3d_View instance |
| `navicube` | `NavCubeOverlay` | The cube widget to sync |

Polling starts immediately on construction.

### Methods

#### `set_interaction_active`

```python
def set_interaction_active(self, active: bool) -> None
```

Call with `True` when the user starts dragging, `False` when they release. Forwarded to the cube for SLERP smoothing and resets the tick counter for an immediate camera read on state change.

#### `teardown`

```python
def teardown(self) -> None
```

Stop polling and disconnect all signals. Call this when the OCC view is being destroyed. After teardown, the sync object holds no references to either the view or the cube.

### Class constants

| Constant | Value | Description |
|:---------|:------|:------------|
| `_TICK_MS` | `16` | Base timer interval (ms) |
| `_INTERACTION_TICKS` | `1` | Poll every tick during interaction |
| `_IDLE_TICKS` | `4` | Poll every 4 ticks when idle |

---

## `VTKNavCubeSync`

```python
from navcube.connectors.vtk import VTKNavCubeSync
```

Same idea as `OCCNavCubeSync`, but for VTK renderers.

### Constructor

```python
VTKNavCubeSync(renderer, navicube)
```

| Parameter | Type | Description |
|:----------|:-----|:------------|
| `renderer` | VTK renderer | A VTK renderer instance |
| `navicube` | `NavCubeOverlay` | The cube widget to sync |

### Methods

#### `set_interaction_active`

```python
def set_interaction_active(self, active: bool) -> None
```

Same behavior as `OCCNavCubeSync.set_interaction_active()`.

#### `teardown`

```python
def teardown(self) -> None
```

Same behavior as `OCCNavCubeSync.teardown()`.

---

## Face and control IDs

These string IDs show up in `hovered_id`, hit-testing, and internal lookups.

### Main faces (6)

`TOP`, `FRONT`, `LEFT`, `BACK`, `RIGHT`, `BOTTOM`

### Edge faces (12)

`FRONT_TOP`, `FRONT_BOTTOM`, `REAR_BOTTOM`, `REAR_TOP`, `REAR_RIGHT`, `FRONT_RIGHT`, `FRONT_LEFT`, `REAR_LEFT`, `TOP_LEFT`, `TOP_RIGHT`, `BOTTOM_RIGHT`, `BOTTOM_LEFT`

### Corner faces (8)

`FTR` (Front-Top-Right), `FTL` (Front-Top-Left), `FBR` (Front-Bottom-Right), `FBL` (Front-Bottom-Left), `RTR` (Rear-Top-Right), `RTL` (Rear-Top-Left), `RBR` (Rear-Bottom-Right), `RBL` (Rear-Bottom-Left)

### Control buttons (8)

| ID | Action | What it does |
|:---|:-------|:-------------|
| `ArrowNorth` | `orbit_u` | Orbit the camera upward |
| `ArrowSouth` | `orbit_d` | Orbit the camera downward |
| `ArrowEast` | `orbit_r` | Orbit the camera rightward |
| `ArrowWest` | `orbit_l` | Orbit the camera leftward |
| `ArrowLeft` | `roll_ccw` | Roll the camera counter-clockwise |
| `ArrowRight` | `roll_cw` | Roll the camera clockwise |
| `ViewMenu` | `home` | Return to home orientation |
| `DotBackside` | `backside` | Flip 180° to the opposite view |
