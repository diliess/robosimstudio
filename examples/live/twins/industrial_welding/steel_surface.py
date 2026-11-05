"""Portable deterministic steel PBR maps and physical-scale mesh UVs.

The maps model rolling marks, mill scale and sparse surface oxidation. They
are authored assets, not measured material data or edited reference photos.
"""

from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter


def make_steel_maps(output: Path):
    output.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(418)
    n = 1024

    def noise(sigma):
        a = gaussian_filter(rng.normal(size=(n, n)), sigma, mode="wrap")
        return a / max(a.std(), 1e-9)

    scale = noise(18)
    grain = noise((0.65, 22))
    fine = noise(0.6)
    oxide = np.clip((noise(35) - 2.0) * 0.07, 0, 0.12)
    value = np.clip(0.52 + 0.011 * scale + 0.007 * grain + 0.003 * fine, 0.36, 0.67)
    rgb = value[..., None] * np.array([0.95, 0.99, 1.02])
    rgb = rgb * (1 - oxide[..., None]) + oxide[..., None] * [0.30, 0.22, 0.15]
    rough = np.clip(0.58 + 0.045 * scale + 0.018 * grain + 0.13 * oxide, 0.4, 0.78)
    height = 0.10 * grain + 0.06 * fine + 0.04 * scale
    dx = (np.roll(height, -1, 1) - np.roll(height, 1, 1)) * 0.20
    dy = (np.roll(height, -1, 0) - np.roll(height, 1, 0)) * 0.20
    normal = np.stack([-dx, dy, np.ones((n, n))], axis=-1)
    normal /= np.linalg.norm(normal, axis=-1)[..., None]
    maps = {"albedo": rgb, "roughness": rough, "normal": normal * 0.5 + 0.5}
    paths = {}
    for name, pixels in maps.items():
        file = output / f"rolled_steel_{name}.png"
        Image.fromarray(np.uint8(np.clip(pixels, 0, 1) * 255)).save(file)
        paths[name] = file
    return paths


def add_uv(shape, dimensions=None, repeats_per_m=2.0):
    from pxr import Sdf, UsdGeom, Vt

    vertices = np.array(shape.GetPointsAttr().Get(), float)
    if dimensions is not None:
        vertices *= np.array(dimensions)
    indices = np.array(shape.GetFaceVertexIndicesAttr().Get())
    counts = shape.GetFaceVertexCountsAttr().Get()
    uv = []
    start = 0
    for count in counts:
        p = vertices[indices[start : start + count]]
        normal = np.cross(p[1] - p[0], p[2] - p[0])
        axis = int(np.argmax(np.abs(normal)))
        keep = [i for i in range(3) if i != axis]
        uv.extend(p[:, keep] * repeats_per_m)
        start += count
    st = UsdGeom.PrimvarsAPI(shape).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "faceVarying")
    st.Set(Vt.Vec2fArray.FromNumpy(np.array(uv, np.float32)))
