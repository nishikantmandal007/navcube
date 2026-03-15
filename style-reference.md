---
layout: default
title: Style Reference
sort: 7
---

# Style Reference

Every field in `NavCubeStyle`, with defaults and examples. This is the single place to control how the cube looks and behaves.

---

## Overview

`NavCubeStyle` is a Python `dataclass`. Pass it when creating the cube, or swap it out at runtime:

```python
from navcube import NavCubeOverlay, NavCubeStyle

# At construction time
style = NavCubeStyle(size=150, theme="dark")
cube = NavCubeOverlay(parent=win, style=style)

# Swap it later
new_style = NavCubeStyle(size=100, theme="light", hover_color=(255, 80, 0, 235))
cube.set_style(new_style)
```

`set_style()` does a full rebuild — rebuilds face geometry, clears the font cache, recalculates DPI scaling, and forces a repaint. It's safe to call at any time.

### Color type

Colors throughout this reference are RGB or RGBA tuples:

```python
Color = Union[Tuple[int, int, int], Tuple[int, int, int, int]]
```

RGB values range from 0–255. The optional fourth element is alpha (0 = fully transparent, 255 = fully opaque). Omitting alpha means fully opaque.

---

## Geometry

These fields control the widget's physical dimensions and the 3D projection.

### `size`

| | |
|:--|:--|
| **Type** | `int` |
| **Default** | `120` |
| **Description** | Base cube drawing area in pixels before DPI scaling. The actual widget size is `size + 2 × padding`. At 96 DPI this is used directly; at higher DPI the widget scales proportionally. |

```python
style = NavCubeStyle(size=180)  # Large cube for a 4K monitor
style = NavCubeStyle(size=80)   # Compact cube for a sidebar
```

### `padding`

| | |
|:--|:--|
| **Type** | `int` |
| **Default** | `10` |
| **Description** | Transparent space on each side of the cube drawing area (before DPI scaling). This is what gives the orbit buttons and shadow room to render without clipping. Total widget side length = `size + 2 × padding`. |

```python
style = NavCubeStyle(padding=20)  # More room for controls and shadow
style = NavCubeStyle(padding=4)   # Tight fit — controls may clip
```

### `scale`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `27.0` |
| **Description** | 3D-to-screen projection scale. Controls how large the cube appears within the drawing area. Higher = cube fills more of the widget. |

```python
style = NavCubeStyle(scale=32.0)  # Bigger cube within the same widget
style = NavCubeStyle(scale=20.0)  # Smaller cube with more margin
```

### `chamfer`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `0.12` |
| **Description** | FreeCAD-style chamfer ratio for edge and corner bevels. Controls how much of each edge is cut away to create the beveled faces. `0.0` = sharp edges, `~0.25` = very large bevels. The default matches FreeCAD's NaviCube look. |

```python
style = NavCubeStyle(chamfer=0.05)  # Minimal bevels
style = NavCubeStyle(chamfer=0.20)  # Heavily beveled
```

---

## Animation and Timing

### `animation_ms`

| | |
|:--|:--|
| **Type** | `int` |
| **Default** | `240` |
| **Description** | Duration of face-click animations in milliseconds. The camera rotates to the target orientation using quaternion SLERP with a quintic ease-in-out curve. |

```python
style = NavCubeStyle(animation_ms=120)  # Snappy
style = NavCubeStyle(animation_ms=600)  # Cinematic
style = NavCubeStyle(animation_ms=1)    # Instant (no animation)
```

### `tick_ms`

| | |
|:--|:--|
| **Type** | `int` |
| **Default** | `16` |
| **Description** | Internal QTimer tick interval in milliseconds. Default is ~60 FPS. The timer only does real work during active animations — idle ticks return immediately. |

```python
style = NavCubeStyle(tick_ms=8)   # Smoother animations, more CPU
style = NavCubeStyle(tick_ms=33)  # Lower CPU usage
```

---

## Thresholds

### `visibility_threshold`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `0.10` |
| **Description** | Dot-product threshold for back-face culling. A face is visible when `dot(face_normal, camera_direction) < threshold`. Lower = fewer faces drawn (only front-facing). Higher = more faces shown (including grazing angles). |

```python
style = NavCubeStyle(visibility_threshold=0.0)   # Only clearly visible faces
style = NavCubeStyle(visibility_threshold=0.25)  # Include grazing-angle faces
```

### `orbit_step_deg`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `15.0` |
| **Description** | Rotation step in degrees for the orbit arrow buttons. Each click rotates the camera by this amount. |

```python
style = NavCubeStyle(orbit_step_deg=5.0)   # Fine-grained
style = NavCubeStyle(orbit_step_deg=45.0)  # Coarse steps
```

### `sync_epsilon`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `1e-3` |
| **Description** | Minimum change in camera direction or up vector (Euclidean norm) before triggering a repaint. Prevents unnecessary redraws when `push_camera()` is called with essentially the same values. |

```python
style = NavCubeStyle(sync_epsilon=1e-5)  # More sensitive — redraw on tiny changes
style = NavCubeStyle(sync_epsilon=1e-2)  # Less sensitive — ignore small jitter
```

### `inactive_opacity`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `0.72` |
| **Description** | Widget opacity when the mouse isn't hovering over it. Ranges from `0.0` (invisible) to `1.0` (fully opaque). Jumps to full opacity on mouse enter and during face-click animations. |

```python
style = NavCubeStyle(inactive_opacity=0.3)  # Nearly invisible when idle
style = NavCubeStyle(inactive_opacity=1.0)  # Always fully visible
```

---

## Lighting

### `light_direction`

| | |
|:--|:--|
| **Type** | `Tuple[float, float, float]` |
| **Default** | `(-0.8, -1.0, -1.8)` |
| **Description** | Direction vector for the Lambertian shading model. Normalized internally. Faces whose normals point opposite to this direction appear brightest. |

```python
style = NavCubeStyle(light_direction=(1.0, -1.0, -2.0))   # Light from top-right
style = NavCubeStyle(light_direction=(0.0, 0.0, -1.0))    # Flat lighting
style = NavCubeStyle(light_direction=(-2.0, 0.0, 0.0))    # Dramatic side light
```

The shading formula: `shade = 0.85 + 0.15 × max(0, dot(face_normal, -light_direction))`. Faces range from 85% to 100% of their base color.

---

## Theme

### `theme`

| | |
|:--|:--|
| **Type** | `str` |
| **Default** | `"auto"` |
| **Description** | Which color palette to use. |

| Value | Behavior |
|:------|:---------|
| `"auto"` | Reads `QApplication.palette()`. If the Window color lightness > 128, uses light theme; otherwise dark. |
| `"light"` | Forces light colors regardless of system palette. |
| `"dark"` | Forces dark colors regardless of system palette. |

```python
style = NavCubeStyle(theme="auto")   # Follow system theme
style = NavCubeStyle(theme="dark")   # Always dark
style = NavCubeStyle(theme="light")  # Always light
```

---

## Light Theme Colors

Used when the resolved theme is "light". All are `Color` (RGB or RGBA tuple).

### `face_color`
Default `(248, 248, 252)` — base fill for the six main faces, subject to Lambertian shading.

### `edge_color`
Default `(210, 210, 215)` — fill for the 12 edge faces. Slightly darker than `face_color` for visual depth.

### `corner_color`
Default `(185, 185, 190)` — fill for the 8 corner faces. Usually the darkest of the three face types.

### `text_color`
Default `(18, 18, 18)` — label text color when the face is not hovered.

### `border_color`
Default `(28, 28, 32)` — outline color for the six main faces.

### `border_secondary_color`
Default `(50, 50, 55)` — outline color for edge and corner faces. Lighter to de-emphasize.

### `control_color`
Default `(186, 186, 192, 120)` — fill for control buttons (orbit arrows, roll arrows, home, backside dot) when not hovered. Semi-transparent by default.

### `control_rim_color`
Default `(105, 105, 110, 170)` — border color for control buttons when not hovered.

### `hover_color`
Default `(0, 148, 255, 235)` — fill for any element when the mouse is over it. Bright blue with slight transparency.

### `hover_text_color`
Default `(255, 255, 255)` — label text color when the face is hovered. White for contrast.

### `dot_color`
Default `(195, 195, 198, 225)` — color of the center dot on the XYZ gizmo (when `show_gizmo=True`).

### `shadow_color`
Default `(0, 0, 0, 42)` — drop shadow behind each face. Very subtle black. Set alpha to 0 to disable shadows entirely.

---

## Dark Theme Colors

Same semantics as the light theme colors, with different defaults tuned for dark backgrounds.

### `face_color_dark`
Default `(155, 160, 178)` — medium blue-gray that reads well against dark backgrounds.

### `edge_color_dark`
Default `(118, 122, 138)`

### `corner_color_dark`
Default `(96, 99, 113)`

### `text_color_dark`
Default `(238, 238, 238)` — near-white for readability.

### `border_color_dark`
Default `(10, 10, 12)` — nearly black.

### `border_secondary_color_dark`
Default `(20, 20, 22)`

### `control_color_dark`
Default `(78, 78, 82, 125)`

### `control_rim_color_dark`
Default `(42, 42, 46, 175)`

### `hover_color_dark`
Default `(0, 148, 255, 235)` — same blue as light theme.

### `hover_text_color_dark`
Default `(255, 255, 255)`

### `dot_color_dark`
Default `(145, 145, 148, 225)`

### `shadow_color_dark`
Default `(0, 0, 0, 78)` — stronger than the light theme shadow since dark backgrounds need more contrast.

---

## Font

### `font_family`

| | |
|:--|:--|
| **Type** | `str` |
| **Default** | `"Arial"` |
| **Description** | Primary font family for face labels. Falls back to `font_fallback` style hint if not available on the system. |

```python
style = NavCubeStyle(font_family="Helvetica Neue")
style = NavCubeStyle(font_family="Segoe UI")
style = NavCubeStyle(font_family="Noto Sans CJK JP")  # For Japanese labels
```

### `font_fallback`

| | |
|:--|:--|
| **Type** | `str` |
| **Default** | `"SansSerif"` |
| **Description** | Qt font style hint used when `font_family` isn't available. Passed to `QFont.setStyleHint()`. |

Valid values: `"SansSerif"`, `"Serif"`, `"Monospace"`, `"TypeWriter"`, `"Cursive"`, `"Fantasy"`, `"System"`.

### `font_weight`

| | |
|:--|:--|
| **Type** | `str` |
| **Default** | `"bold"` |
| **Description** | Font weight for face labels. |

Valid values: `"thin"`, `"extralight"`, `"light"`, `"normal"`, `"medium"`, `"demibold"`, `"bold"`, `"extrabold"`, `"black"`.

### `label_max_width_ratio`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `0.70` |
| **Description** | Max fraction of the 200-unit virtual label canvas that text may occupy horizontally. Font size auto-scales to fit. |

### `label_max_height_ratio`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `0.45` |
| **Description** | Max fraction of the 200-unit virtual label canvas that text may occupy vertically. Works with `label_max_width_ratio` — the smaller computed size wins. |

### `min_font_size`

| | |
|:--|:--|
| **Type** | `float` |
| **Default** | `40.0` |
| **Description** | Minimum font point size in the virtual label space. Prevents long labels from becoming unreadably tiny — text may overflow slightly rather than disappear. |

---

## Labels

### `labels`

| | |
|:--|:--|
| **Type** | `Dict[str, str]` |
| **Default** | `{"TOP": "TOP", "FRONT": "FRONT", "LEFT": "LEFT", "BACK": "BACK", "RIGHT": "RIGHT", "BOTTOM": "BOTTOM"}` |
| **Description** | Display text for each main face. Only the six main faces have labels — edges and corners don't. Keys must be exactly: `TOP`, `FRONT`, `LEFT`, `BACK`, `RIGHT`, `BOTTOM`. |

```python
# German labels
style = NavCubeStyle(labels={
    "TOP": "OBEN", "FRONT": "VORNE", "LEFT": "LINKS",
    "BACK": "HINTEN", "RIGHT": "RECHTS", "BOTTOM": "UNTEN",
})

# Japanese labels
style = NavCubeStyle(
    font_family="Noto Sans CJK JP",
    labels={
        "TOP": "\u4e0a", "FRONT": "\u524d", "LEFT": "\u5de6",
        "BACK": "\u5f8c", "RIGHT": "\u53f3", "BOTTOM": "\u4e0b",
    },
)
```

---

## Controls

### `show_controls`

| | |
|:--|:--|
| **Type** | `bool` |
| **Default** | `True` |
| **Description** | Whether to show the orbit buttons, roll arrows, home button, and backside dot. `False` hides all controls, leaving just the cube faces. |

```python
style = NavCubeStyle(show_controls=False)  # Cube faces only
```

The control elements:
- **ArrowNorth / ArrowSouth / ArrowEast / ArrowWest** — orbit buttons, each steps by `orbit_step_deg`
- **ArrowLeft / ArrowRight** — roll arrows, rotate around the view axis
- **ViewMenu** — home button, snaps back to the home orientation
- **DotBackside** — flips 180° to the opposite view

### `show_gizmo`

| | |
|:--|:--|
| **Type** | `bool` |
| **Default** | `False` |
| **Description** | Whether to show the XYZ axis gizmo in the lower-left corner. Three colored lines (X=red, Y=green, Z=blue) oriented to match the current camera. |

```python
style = NavCubeStyle(show_gizmo=True)
```

### `gizmo_x_color`
Default `(215, 52, 52)` — X-axis line color (red).

### `gizmo_y_color`
Default `(52, 195, 52)` — Y-axis line color (green).

### `gizmo_z_color`
Default `(55, 115, 255)` — Z-axis line color (blue).

### `gizmo_font_size`

| | |
|:--|:--|
| **Type** | `int` |
| **Default** | `9` |
| **Description** | Font size in points for the X/Y/Z labels on the gizmo. |

---

## Border Widths

### `border_width_main`
Default `2.0` — pen width in pixels for main face outlines.

```python
style = NavCubeStyle(border_width_main=3.5)  # Thick borders
style = NavCubeStyle(border_width_main=0.0)  # No borders
```

### `border_width_secondary`
Default `1.2` — pen width for edge and corner face outlines. Thinner than main for visual hierarchy.

### `control_border_width`
Default `1.2` — pen width for control button borders.

---

## Shadow

### `shadow_offset_x`
Default `1.8` — horizontal shadow offset in pixels. Positive = right.

### `shadow_offset_y`
Default `2.3` — vertical shadow offset in pixels. Positive = down.

```python
style = NavCubeStyle(shadow_color=(0, 0, 0, 0))  # No shadow

style = NavCubeStyle(  # Dramatic shadow
    shadow_offset_x=4.0,
    shadow_offset_y=5.0,
    shadow_color=(0, 0, 0, 100),
)
```

---

## Complete field reference

| Field | Type | Default | Section |
|:------|:-----|:--------|:--------|
| `size` | `int` | `120` | Geometry |
| `padding` | `int` | `10` | Geometry |
| `scale` | `float` | `27.0` | Geometry |
| `chamfer` | `float` | `0.12` | Geometry |
| `animation_ms` | `int` | `240` | Animation |
| `tick_ms` | `int` | `16` | Animation |
| `visibility_threshold` | `float` | `0.10` | Thresholds |
| `orbit_step_deg` | `float` | `15.0` | Thresholds |
| `sync_epsilon` | `float` | `1e-3` | Thresholds |
| `inactive_opacity` | `float` | `0.72` | Thresholds |
| `light_direction` | `Tuple[float,float,float]` | `(-0.8, -1.0, -1.8)` | Lighting |
| `theme` | `str` | `"auto"` | Theme |
| `face_color` | `Color` | `(248, 248, 252)` | Light Colors |
| `edge_color` | `Color` | `(210, 210, 215)` | Light Colors |
| `corner_color` | `Color` | `(185, 185, 190)` | Light Colors |
| `text_color` | `Color` | `(18, 18, 18)` | Light Colors |
| `border_color` | `Color` | `(28, 28, 32)` | Light Colors |
| `border_secondary_color` | `Color` | `(50, 50, 55)` | Light Colors |
| `control_color` | `Color` | `(186, 186, 192, 120)` | Light Colors |
| `control_rim_color` | `Color` | `(105, 105, 110, 170)` | Light Colors |
| `hover_color` | `Color` | `(0, 148, 255, 235)` | Light Colors |
| `hover_text_color` | `Color` | `(255, 255, 255)` | Light Colors |
| `dot_color` | `Color` | `(195, 195, 198, 225)` | Light Colors |
| `shadow_color` | `Color` | `(0, 0, 0, 42)` | Light Colors |
| `face_color_dark` | `Color` | `(155, 160, 178)` | Dark Colors |
| `edge_color_dark` | `Color` | `(118, 122, 138)` | Dark Colors |
| `corner_color_dark` | `Color` | `(96, 99, 113)` | Dark Colors |
| `text_color_dark` | `Color` | `(238, 238, 238)` | Dark Colors |
| `border_color_dark` | `Color` | `(10, 10, 12)` | Dark Colors |
| `border_secondary_color_dark` | `Color` | `(20, 20, 22)` | Dark Colors |
| `control_color_dark` | `Color` | `(78, 78, 82, 125)` | Dark Colors |
| `control_rim_color_dark` | `Color` | `(42, 42, 46, 175)` | Dark Colors |
| `hover_color_dark` | `Color` | `(0, 148, 255, 235)` | Dark Colors |
| `hover_text_color_dark` | `Color` | `(255, 255, 255)` | Dark Colors |
| `dot_color_dark` | `Color` | `(145, 145, 148, 225)` | Dark Colors |
| `shadow_color_dark` | `Color` | `(0, 0, 0, 78)` | Dark Colors |
| `font_family` | `str` | `"Arial"` | Font |
| `font_fallback` | `str` | `"SansSerif"` | Font |
| `font_weight` | `str` | `"bold"` | Font |
| `label_max_width_ratio` | `float` | `0.70` | Font |
| `label_max_height_ratio` | `float` | `0.45` | Font |
| `min_font_size` | `float` | `40.0` | Font |
| `labels` | `Dict[str, str]` | `{"TOP":"TOP", ...}` | Labels |
| `show_controls` | `bool` | `True` | Controls |
| `show_gizmo` | `bool` | `False` | Controls |
| `gizmo_x_color` | `Color` | `(215, 52, 52)` | Controls |
| `gizmo_y_color` | `Color` | `(52, 195, 52)` | Controls |
| `gizmo_z_color` | `Color` | `(55, 115, 255)` | Controls |
| `gizmo_font_size` | `int` | `9` | Controls |
| `border_width_main` | `float` | `2.0` | Border Widths |
| `border_width_secondary` | `float` | `1.2` | Border Widths |
| `control_border_width` | `float` | `1.2` | Border Widths |
| `shadow_offset_x` | `float` | `1.8` | Shadow |
| `shadow_offset_y` | `float` | `2.3` | Shadow |

---

## Example: dark blue theme

```python
style = NavCubeStyle(
    theme="dark",
    face_color_dark=(40, 60, 120),
    edge_color_dark=(30, 45, 90),
    corner_color_dark=(20, 30, 60),
    text_color_dark=(200, 220, 255),
    border_color_dark=(10, 15, 40),
    border_secondary_color_dark=(15, 20, 50),
    hover_color_dark=(80, 160, 255, 235),
    hover_text_color_dark=(255, 255, 255),
    control_color_dark=(30, 50, 100, 120),
    control_rim_color_dark=(20, 35, 70, 170),
    shadow_color_dark=(0, 0, 20, 90),
)
```

## Example: minimal wireframe theme

```python
style = NavCubeStyle(
    theme="light",
    face_color=(255, 255, 255),
    edge_color=(255, 255, 255),
    corner_color=(255, 255, 255),
    text_color=(0, 0, 0),
    border_color=(0, 0, 0),
    border_secondary_color=(0, 0, 0),
    border_width_main=1.5,
    border_width_secondary=0.8,
    shadow_color=(0, 0, 0, 0),  # No shadow
    hover_color=(230, 230, 230, 255),
    hover_text_color=(0, 0, 0),
    show_controls=False,
    inactive_opacity=1.0,
)
```

## Runtime style changes

```python
from navcube import NavCubeOverlay, NavCubeStyle

cube = NavCubeOverlay(parent=win)

# Switch to a custom style
dark_style = NavCubeStyle(theme="dark", size=150)
cube.set_style(dark_style)

# Reset to defaults
cube.set_style(NavCubeStyle())
```

Internally, `set_style()` does all of this in one shot:
1. Stores the new style
2. Recomputes derived values (scale, light direction, thresholds, etc.)
3. Clears the font size cache
4. Rebuilds all face geometry (in case `chamfer` changed)
5. Recalculates DPI scaling and widget size
6. Updates the timer interval
7. Forces a full repaint
