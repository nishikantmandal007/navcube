---
layout: default
title: Coordinate Systems
sort: 5
---

# Coordinate Systems

NavCube needs to know which way "up" is in your 3D world. This page explains how that works and how to configure it for your engine.

---

## Z-up vs Y-up

The 3D world can't agree on which axis points up, and neither can the apps built on it:

| Convention | Used by | Up axis | Forward axis |
|:-----------|:--------|:--------|:-------------|
| **Z-up** | OCC, FreeCAD, Blender (world), AutoCAD, Civil engineering | +Z | -Y or +X |
| **Y-up** | Three.js, glTF, Unity, Unreal, most game engines | +Y | -Z or +X |

NavCube renders internally in **Z-up** space. The face labeled "TOP" always corresponds to +Z in the internal coordinate system. The `_WORLD_ROT` matrix is what bridges between that internal space and your application's world space.

---

## How `_WORLD_ROT` works

`_WORLD_ROT` is a 3×3 rotation matrix that maps **from** NavCube's internal Z-up space **to** your application's world space:

```
world_vector = _WORLD_ROT @ navicube_vector
navicube_vector = _WORLD_ROT.T @ world_vector
```

### Default: Z-up (identity)

For Z-up applications (OCC, FreeCAD, Blender), there's nothing to do — the identity matrix is the default:

```python
_WORLD_ROT = np.eye(3)  # identity — this is the default
```

```
Internal Z-up          Your world (Z-up)
    +Z (TOP)    --->       +Z (TOP)
    +Y (BACK)   --->       +Y (BACK)
    +X (RIGHT)  --->       +X (RIGHT)
```

### Y-up transformation

For Y-up engines, you need to swap Y and Z (with a sign flip to stay right-handed):

```python
_WORLD_ROT = np.array([
    [1,  0,  0],
    [0,  0, -1],
    [0,  1,  0],
])
```

```
Internal Z-up          Your world (Y-up)
    +Z (TOP)    --->       +Y (TOP)
    +Y (BACK)   --->       -Z (BACK)
    +X (RIGHT)  --->       +X (RIGHT)
```

This rotates -90° around the X axis, turning Z-up into Y-up.

---

## Setting up Y-up via subclassing

The recommended way to use NavCube with a Y-up engine is to subclass `NavCubeOverlay` and set `_WORLD_ROT` as a class attribute:

```python
import numpy as np
from navcube import NavCubeOverlay

class YUpNaviCube(NavCubeOverlay):
    """NaviCube for Y-up coordinate systems (Three.js, Unity, etc.)."""
    _WORLD_ROT = np.array([
        [1,  0,  0],
        [0,  0, -1],
        [0,  1,  0],
    ], dtype=float)

# Usage — no manual coordinate conversion needed
cube = YUpNaviCube(parent=viewport)
cube.viewOrientationRequested.connect(on_orient)

# push_camera receives Y-up vectors directly
cube.push_camera(dx, dy, dz, ux, uy, uz)
```

The `__init_subclass__` hook gives each subclass its own independent copy of `_WORLD_ROT`, so changing one subclass won't affect others.

**Both `push_camera()` and `viewOrientationRequested` work in your world space.** NavCube converts to/from internal Z-up space internally. You never need to transform coordinates manually.

---

## Sign convention

This is the most important contract in the library. Getting the sign wrong will make the cube rotate backwards when you orbit.

### `push_camera` — INWARD direction

```
push_camera(dx, dy, dz, ux, uy, uz)
             ↑                ↑
        INWARD dir        Up vector
       (eye → scene)
```

The direction vector points from the camera eye toward the scene center. This matches OCC's `Camera().Direction()`.

### `viewOrientationRequested` — OUTWARD direction

```
viewOrientationRequested(dx, dy, dz, ux, uy, uz)
                          ↑                ↑
                     OUTWARD dir       Up vector
                    (scene → eye)
```

The direction vector points from the scene center toward the camera eye. This matches OCC's `V3d_View.SetProj()`.

### The easy way to remember this

> **Read inward, write outward.**
>
> - You **read** your camera and push the direction **inward** (eye toward scene).
> - NavCube **writes** orientation requests **outward** (scene toward eye).

NavCube handles the negation internally. You'll never need to flip signs in your code.

---

## Sign convention cheat sheet

```
┌─────────────────────────────────────────────────────────┐
│                    YOUR RENDERER                        │
│                                                         │
│  Camera state:                                          │
│    direction = eye → scene   (INWARD)                   │
│    up        = camera up vector                         │
│                                                         │
│         │ push_camera(dir, up)                          │
│         ▼                                               │
│  ┌─────────────────────────────┐                        │
│  │     NavCubeOverlay          │                        │
│  │                             │                        │
│  │  Internal: negates dir      │                        │
│  │  for emission               │                        │
│  └─────────────────────────────┘                        │
│         │ viewOrientationRequested(-dir, up)             │
│         ▼                                               │
│                                                         │
│  Update camera:                                         │
│    SetProj(dx, dy, dz)  ← OUTWARD (scene → eye)        │
│    SetUp(ux, uy, uz)                                    │
│    Redraw()                                             │
└─────────────────────────────────────────────────────────┘
```

---

## Axis diagrams

### Z-up coordinate system (default)

```
        +Z (TOP)
         │
         │
         │
         │
         ┼──────── +X (RIGHT)
        ╱
       ╱
      ╱
    +Y (BACK)


    Face mapping:
    ┌─────────────────────────┐
    │  +Z normal  →  TOP      │
    │  -Z normal  →  BOTTOM   │
    │  -Y normal  →  FRONT    │
    │  +Y normal  →  BACK     │
    │  +X normal  →  RIGHT    │
    │  -X normal  →  LEFT     │
    └─────────────────────────┘
```

### Y-up coordinate system (with `_WORLD_ROT`)

```
        +Y (TOP)
         │
         │
         │
         │
         ┼──────── +X (RIGHT)
        ╱
       ╱
      ╱
    +Z (FRONT)


    World-space face mapping after _WORLD_ROT:
    ┌─────────────────────────┐
    │  +Y normal  →  TOP      │
    │  -Y normal  →  BOTTOM   │
    │  +Z normal  →  FRONT    │
    │  -Z normal  →  BACK     │
    │  +X normal  →  RIGHT    │
    │  -X normal  →  LEFT     │
    └─────────────────────────┘
```

### Default ISO view

The default camera orientation is an isometric view looking at the FRONT-RIGHT-TOP corner:

```
        +Z
         │  ╲
         │    ╲  Camera looks from here
         │      ●  (iso view)
         │    ╱
         ┼──╱──── +X
        ╱
       ╱
    +Y

    Default inward direction: normalize(-1, +1, -1)
    Default up vector: (0, 0, +1)
```

The camera looks toward the origin from `(-1, +1, -1)` (normalized), with Z as the up vector. This puts FRONT, RIGHT, and TOP all partially visible.

---

## More `_WORLD_ROT` examples

### Left-handed Y-up (rare)

```python
class LeftHandedYUpNaviCube(NavCubeOverlay):
    _WORLD_ROT = np.array([
        [1,  0,  0],
        [0,  0,  1],
        [0, -1,  0],
    ], dtype=float)
```

### Swapped X and Y

```python
class SwappedXYNaviCube(NavCubeOverlay):
    _WORLD_ROT = np.array([
        [0,  1,  0],
        [1,  0,  0],
        [0,  0,  1],
    ], dtype=float)
```

### Arbitrary rotation

You can use any valid 3×3 rotation matrix — as long as it's orthonormal (unit-length columns, mutually perpendicular). Use any matrix that isn't orthonormal and things will look wrong.
