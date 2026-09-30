"""The user's model library: STL files copied into the application data folder plus models.json.

models.json (a list under "models"), per model:
    id, name, file, builtin, scale [x, y, z], pivot [x, y, z] | null, forward_axis, up_axis
    (a copy of a built-in model has "base" instead of "file").
Loading validates every entry and never raises: a bad entry gets defaults, a missing or unreadable
file shows a placeholder mesh (ModelRegistry.mesh). Editing happens on a LibraryDraft and only
`ModelLibrary.commit(draft)` makes it real ("Зберегти" in the manager dialog).
No Qt in here.
"""

import copy
import json
import re
import shutil
from pathlib import Path

import numpy as np

from ..frames import board_rotation
from .models import (
    AXES,
    BUILTIN,
    DEFAULT_COLOR,
    DEFAULT_SIZE_M,
    ModelRegistry,
    ModelSpec,
    axes_from_matrix,
    axes_rotation,
    load_stl,
    surface_centroid,
)

DEFAULT_FORWARD, DEFAULT_UP = "+X", "+Z"  # what most STL files look like


def _vec3(value, default):
    try:
        v = tuple(float(x) for x in value)
        if len(v) == 3 and all(np.isfinite(v)):
            return v
    except (TypeError, ValueError):
        pass
    return default


def _slug(text: str) -> str:
    s = re.sub(r"[^0-9a-zA-Zа-яА-ЯіїєґІЇЄҐ_-]+", "-", text.strip()).strip("-").lower()
    return s or "model"


def spec_to_json(s: ModelSpec) -> dict:
    d = {
        "id": s.key,
        "name": s.display_name,
        "file": s.file,
        "builtin": False,
        "scale": list(s.scale_xyz or (1.0, 1.0, 1.0)),
        "pivot": None if s.center is None else list(s.center),
        "forward_axis": s.forward_axis or DEFAULT_FORWARD,
        "up_axis": s.up_axis or DEFAULT_UP,
    }
    if s.base:
        d["base"] = s.base
    return d


def spec_from_json(d) -> ModelSpec | None:
    """None if the entry cannot be used at all (no id / name); other faults get defaults."""
    if not isinstance(d, dict) or not isinstance(d.get("id"), str) or not d["id"]:
        return None
    if d["id"] in BUILTIN:
        return None  # built-in models are not stored
    name = d.get("name") if isinstance(d.get("name"), str) and d.get("name") else d["id"]
    fwd, up = d.get("forward_axis"), d.get("up_axis")
    try:
        if fwd not in AXES or up not in AXES:
            raise ValueError
        axes_rotation(fwd, up)
    except ValueError:  # unknown or collinear axes
        fwd, up = DEFAULT_FORWARD, DEFAULT_UP
    scale = _vec3(d.get("scale"), (1.0, 1.0, 1.0))
    if min(scale) <= 0:
        scale = (1.0, 1.0, 1.0)
    base = d.get("base") if d.get("base") in BUILTIN else None
    file = d.get("file") if isinstance(d.get("file"), str) else None
    pivot = d.get("pivot")
    return ModelSpec(
        key=d["id"],
        display_name=name,
        file=None if base else file,
        scale_xyz=scale,
        center=None if pivot is None else _vec3(pivot, None),
        color=DEFAULT_COLOR,
        forward_axis=fwd,
        up_axis=up,
        base=base,
        broken=False,
    )


class LibraryDraft:
    """Changes not saved yet. Files of added models are already copied (removed on discard);
    files of deleted models stay until commit."""

    def __init__(self, library: "ModelLibrary"):
        self.lib = library
        self.specs: dict[str, ModelSpec] = {
            k: copy.deepcopy(s) for k, s in library.all_specs().items()
        }
        self.added_files: list[Path] = []
        self.removed_files: list[str] = []

    # ----- queries -----

    def keys(self) -> list[str]:
        return list(self.specs)

    def registry(self) -> ModelRegistry:
        """A registry of the draft (for the preview in the dialog)."""
        return ModelRegistry(self.lib.dir, self.lib.size, specs=list(self.specs.values()))

    # ----- edits -----

    def _unique_key(self, base: str) -> str:
        key, n = base, 1
        while key in self.specs or key in BUILTIN:
            n += 1
            key = f"{base}-{n}"
        return key

    def _copy_in(self, src: Path) -> str:
        self.lib.dir.mkdir(parents=True, exist_ok=True)
        stem, n = _slug(src.stem), 1
        dest = self.lib.dir / f"{stem}.stl"
        while dest.exists() or dest in self.added_files:
            n += 1
            dest = self.lib.dir / f"{stem}-{n}.stl"
        shutil.copyfile(src, dest)
        self.added_files.append(dest)
        return dest.name

    def add(self, path, name: str | None = None, key: str | None = None) -> str:
        """Copy an STL into the library. Raises ValueError/OSError if it cannot be read."""
        path = Path(path)
        tris = load_stl(path)  # validates before anything is copied
        file = self._copy_in(path)
        key = self._unique_key(key or _slug(name or path.stem))
        self.specs[key] = ModelSpec(
            key=key,
            display_name=name or path.stem,
            file=file,
            scale_xyz=(1.0, 1.0, 1.0),
            center=tuple(float(x) for x in surface_centroid(tris)),
            forward_axis=DEFAULT_FORWARD,
            up_axis=DEFAULT_UP,
        )
        return key

    def duplicate(self, key: str) -> str:
        src = self.specs[key]
        new = copy.deepcopy(src)
        new.key = self._unique_key(f"{src.key}-copy")
        new.display_name = f"{src.display_name} (копія)"
        new.builtin = False
        if src.file:
            new.file = self._copy_in(self.lib.dir / src.file)
        elif src.builtin:
            new.base = src.key  # the copy uses the built-in geometry
            new.forward_axis, new.up_axis = "+X", "-Z"  # built-in geometry is already FRD
            new.scale_xyz = new.scale_xyz or (1.0, 1.0, 1.0)
        new.broken = False
        self.specs[new.key] = new
        return new.key

    def delete(self, key: str):
        if self.specs[key].builtin:
            raise ValueError("a built-in model cannot be deleted")
        spec = self.specs.pop(key)
        if spec.file:
            self.removed_files.append(spec.file)

    def update(self, key: str, **fields):
        spec = self.specs[key]
        if spec.builtin:
            raise ValueError("a built-in model cannot be edited")
        for k, v in fields.items():
            setattr(spec, k, v)

    def discard(self):
        for f in self.added_files:
            f.unlink(missing_ok=True)
        self.added_files = []


class ModelLibrary:
    def __init__(self, directory, seed_dir=None, size=DEFAULT_SIZE_M):
        self.dir = Path(directory)
        self.size = size
        self.json_path = self.dir / "models.json"
        self.user: dict[str, ModelSpec] = {}
        self.problems: list[str] = []  # what was wrong while loading (for the log)
        self.load(seed_dir)

    # ----- loading -----

    def load(self, seed_dir=None):
        self.user = {}
        self.dir.mkdir(parents=True, exist_ok=True)
        if not self.json_path.exists():
            if seed_dir:
                self._seed(Path(seed_dir))
            return
        try:
            data = json.loads(self.json_path.read_text(encoding="utf-8"))
            entries = data["models"]
            if not isinstance(entries, list):
                raise TypeError("models is not a list")
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.problems.append(f"models.json unreadable ({e}); started again")
            self.json_path.replace(self.json_path.with_suffix(".json.bad"))
            if seed_dir:
                self._seed(Path(seed_dir))
            return
        for entry in entries:
            spec = spec_from_json(entry)
            if spec is None or spec.key in self.user:
                self.problems.append(f"skipped entry {entry!r}")
                continue
            self.user[spec.key] = spec

    def _seed(self, seed_dir: Path):
        """First run: the STL files shipped with the program become normal library entries."""
        if not seed_dir.is_dir():
            return
        legacy = ModelRegistry(seed_dir, self.size)
        draft = LibraryDraft(self)
        for spec in legacy.specs.values():
            if spec.file is None:
                continue
            try:
                new = draft.add(seed_dir / spec.file, spec.display_name, spec.key)
            except (OSError, ValueError):
                continue
            try:
                f, u = axes_from_matrix(board_rotation(*spec.rotation_deg))
            except ValueError:
                f, u = DEFAULT_FORWARD, DEFAULT_UP
            draft.update(new, forward_axis=f, up_axis=u)
            if spec.center is not None:
                draft.update(new, center=tuple(spec.center))
            if spec.scale != 1.0:
                draft.update(new, scale_xyz=(spec.scale,) * 3)
        self.commit(draft)

    # ----- reading -----

    def all_specs(self) -> dict[str, ModelSpec]:
        out = {k: copy.deepcopy(v[0]) for k, v in BUILTIN.items()}
        out.update(self.user)
        return out

    def registry(self) -> ModelRegistry:
        return ModelRegistry(self.dir, self.size, specs=list(self.all_specs().values()))

    def draft(self) -> LibraryDraft:
        return LibraryDraft(self)

    # ----- saving -----

    def save(self):
        data = {"version": 1, "models": [spec_to_json(s) for s in self.user.values()]}
        tmp = self.json_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        tmp.replace(self.json_path)

    def commit(self, draft: LibraryDraft) -> ModelRegistry:
        """Make a draft the library: write models.json, drop files nobody uses any more."""
        self.user = {k: s for k, s in draft.specs.items() if not s.builtin}
        self.save()
        used = {s.file for s in self.user.values() if s.file}
        for name in draft.removed_files:  # only files of models that were deleted in the dialog
            if name not in used:
                (self.dir / name).unlink(missing_ok=True)
        draft.added_files = []
        return self.registry()
