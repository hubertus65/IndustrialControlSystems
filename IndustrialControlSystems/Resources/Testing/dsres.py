# Copyright © 2026 Model Based Innovation LLC.
# SPDX-License-Identifier: BSD-3-Clause -- the text is BSD-3-Clause.txt in the export bundle
# (ModCheck/Resources/Testing/ in ModCheck, Resources/Testing/ in a library carrying a copy).

"""
Read and write a Dymola/OpenModelica "dsres" result file (.mat, MATLAB v4)
without either tool.

Both tools write the same layout, which Dymola defined and OpenModelica
adopted: `Aclass` (four text rows; the fourth says `binTrans` when every other
matrix is stored transposed, which both tools do), `name` (one variable name
per row, padded), `dataInfo` (per variable: which data matrix, which column --
negative for a sign-flipped alias -- and two flags), `data_1` (parameters and
constants: two samples, start and stop) and `data_2` (everything that varies:
one column per time sample). `Time` (Dymola) or `time` (OpenModelica) is the
first column of `data_2`.

Reading it here rather than through `dymola.readTrajectory()` /
`OMPython.getSolutions()` matters for two reasons: the Dymola session is the
expensive licensed thing and should not be needed to read a file back, and
OMPython's reader is one more layer over exactly this format. It also means a
parameter can be read back after a run -- which is how a source finds out
whether a modifier was honoured or silently ignored as structural
(Planning/OPENMODELICA_PLAN.md section 4, DYMOLA_PLAN.md section 3).

`write()` is the other direction: trajectories on one time grid to a file
Dymola, OMEdit, OpenModelica's diffSimulationResults, MATLAB and scipy read.
It writes the plain layout -- every signal its own column of `data_2`, no
aliases, `data_1` holding only the time interval -- byte by byte, with no
scipy. This module depends on numpy alone for writing (scipy is imported only
by the unmapped reader), which is what lets it be copied unchanged into the
export bundle, `Resources/Testing/dsres.py` (Documentation.md section 5f).
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np


class DsResult:
    """One loaded result file. `names` is every variable; `trajectory(name)`
    returns its samples over the `time` grid; `parameter(name)` its constant
    value; `time` is the time column under ModCheck's lower-case name."""

    def __init__(self, path: str | Path, mapped: bool = False):
        """`mapped`: the data matrix is memory-mapped, not loaded, and each trajectory is
        copied out of it on request -- a year of a check model's result is 1.5 GB, and
        loading it whole doubles that (_v4_matrices)."""
        self.path = Path(path)
        self._mapped = mapped
        if mapped:
            m = _v4_mapped(self.path)
        else:
            import scipy.io
            m = scipy.io.loadmat(str(self.path), matlab_compatible=True, chars_as_strings=False)
        aclass = _rows_as_text(m["Aclass"])
        transposed = any("binTrans" in row for row in aclass)
        self.names = _rows_as_text(m["name"] if not transposed else m["name"].T)
        info = m["dataInfo"]
        self._info = (info.T if transposed else info).astype(int)
        self._data = {}
        for key in ("data_1", "data_2"):
            if key in m:
                d = m[key] if mapped else np.asarray(m[key], dtype=float)
                self._data[key] = d.T if transposed else d
        self._index = {n: i for i, n in enumerate(self.names)}
        time_name = "Time" if "Time" in self._index else "time"
        self.time_name = time_name
        self.time = np.array(self._column(time_name), dtype=float) if mapped else self._column(time_name)

    # ---------------------------------------------------------------- access
    def _column(self, name: str) -> np.ndarray:
        i = self._index[name]
        matrix, col = int(self._info[i][0]), int(self._info[i][1])
        if matrix == 0:            # the time variable itself
            matrix, col = 2, 1
        d = self._data[f"data_{matrix}"]
        values = d[:, abs(col) - 1]
        return -values if col < 0 else values

    def is_parameter(self, name: str) -> bool:
        return int(self._info[self._index[name]][0]) == 1

    def trajectory(self, name: str) -> np.ndarray:
        """Samples over `time`. A parameter is broadcast to the grid so a case
        that asks for one (they do -- see run_metrics' `optimizer.objective`
        precedent) gets an array of the right length rather than two values."""
        values = self._column(name)
        if self.is_parameter(name):
            return np.full(self.time.shape, float(values[-1]))
        return np.array(values, dtype=float) if self._mapped else np.asarray(values, dtype=float)

    def parameter(self, name: str) -> float:
        return float(self._column(name)[-1])

    def __contains__(self, name: str) -> bool:
        return name in self._index

    def trajectories(self, names) -> dict[str, np.ndarray]:
        """`{name: samples}` for these names plus "time", in ModCheck's shape.
        A name the file does not carry is left out -- the core reports it
        MISSING, which is the honest outcome and has a reporting path."""
        out = {"time": np.asarray(self.time, dtype=float)}
        for n in names:
            if n in self._index and n not in ("time", "Time"):
                out[n] = self.trajectory(n)
        return out


_V4_TYPES = {0: "f8", 1: "f4", 2: "i4", 3: "i2", 4: "u2", 5: "u1"}


def _v4_matrices(path: Path) -> dict[str, tuple[np.dtype, int, int, int]]:
    """Every matrix of a MATLAB v4 file: name -> (dtype, rows, columns, byte offset of its data).
    Each matrix is a 20-byte header (type MOPT, rows, columns, imaginary flag, name length),
    the name, then the data column by column."""
    out = {}
    size = path.stat().st_size
    with open(path, "rb") as fh:
        off = 0
        while off < size:
            fh.seek(off)
            raw = fh.read(20)
            head = np.frombuffer(raw, "<i4")
            if not 0 <= int(head[0]) < 2000:
                head = np.frombuffer(raw, ">i4")
            typ, rows, cols, imag, namelen = (int(x) for x in head)
            m, p_ = typ // 1000, (typ // 10) % 10
            if m not in (0, 1) or p_ not in _V4_TYPES:
                raise ValueError(f"{path}: not a MATLAB v4 matrix at byte {off} (type {typ})")
            name = fh.read(namelen).rstrip(b"\x00").decode("ascii")
            dt = np.dtype(("<" if m == 0 else ">") + _V4_TYPES[p_])
            data = off + 20 + namelen
            out[name] = (dt, rows, cols, data)
            off = data + rows * cols * dt.itemsize * (2 if imag else 1)
    return out


def _v4_mapped(path: Path) -> dict[str, np.ndarray]:
    """The matrices as scipy.io.loadmat gives them (rows x columns), data_2 memory-mapped and
    the rest read: a column-major matrix is the transpose of a row-major (columns x rows) one."""
    out = {}
    for name, (dt, rows, cols, off) in _v4_matrices(path).items():
        if name == "data_2":
            out[name] = np.memmap(path, dtype=dt, mode="r", offset=off, shape=(cols, rows)).T
        else:
            out[name] = np.fromfile(path, dtype=dt, count=rows * cols, offset=off).reshape(cols, rows).T
    return out


def _rows_as_text(a) -> list[str]:
    """A MATLAB char matrix as a list of stripped strings, whether scipy gave
    it back as characters or as their codes."""
    a = np.asarray(a)
    rows = []
    for row in a:
        if row.dtype.kind in "US":
            s = "".join(str(c) for c in row)
        else:
            s = "".join(chr(int(c)) for c in row)
        rows.append(s.rstrip("\x00 "))
    return rows


def read(path: str | Path, mapped: bool = False) -> DsResult:
    return DsResult(path, mapped)


# ---------------------------------------------------------------------- write
def v4_matrix(name: str, data, columns: bool = False) -> bytes:
    """One MATLAB v4 matrix, little-endian: the 20-byte header, the name, the data column by
    column. Strings are one per row, or one per column with `columns` (the layout of
    `name` and `description` in a binTrans file)."""
    data = np.asarray(data)
    if data.dtype.kind in "US":
        width = max([len(s) for s in data] + [1])
        data = np.array([[ord(c) for c in s.ljust(width)] for s in data], dtype="u1")
        data = data.T if columns else data
        typ = 51                                     # text, uint8
    else:
        typ = {np.dtype("f8"): 0, np.dtype("f4"): 10, np.dtype("i4"): 20}[data.dtype]
    rows, cols = data.shape
    head = struct.pack("<5i", typ, rows, cols, 0, len(name) + 1)
    return head + name.encode("ascii") + b"\0" + np.asfortranarray(data).tobytes(order="F")


def write(path: str | Path, signals: dict, time_name: str = "time",
          descriptions: dict | None = None) -> Path:
    """Write `signals` -- {name: 1-D array}, one of them "time" (or `time_name`), all the
    same length -- as a dsres file. Time goes first under `time_name` ("time" as
    OpenModelica writes it, "Time" as Dymola does); every other signal follows in the
    order given, as float64. Repeated time points (an event's two samples) are kept.
    `descriptions` adds a description per name, e.g. "[unit]". Returns the path."""
    path = Path(path)
    key = "time" if "time" in signals else time_name
    if key not in signals:
        raise ValueError("write(): no 'time' signal")
    t = np.asarray(signals[key], dtype="f8").ravel()
    names = [n for n in signals if n not in ("time", "Time")]
    cols = [t]
    for n in names:
        y = np.asarray(signals[n], dtype="f8").ravel()
        if y.shape != t.shape:
            raise ValueError(f"write(): {n!r} has {y.size} samples, time has {t.size}")
        cols.append(y)
    if not t.size:
        raise ValueError("write(): no samples")
    desc = descriptions or {}
    all_names = [time_name] + names
    texts = [desc.get(key, "Time in seconds" if key in ("time", "Time") else "")] + \
        [desc.get(n, "") for n in names]
    if any(ord(c) > 127 for s in all_names + texts for c in s):
        raise ValueError("write(): names and descriptions must be ASCII")
    # dataInfo per variable: data matrix (0 = the time variable, 2 = data_2), column, and
    # the two flags (0 = interpolate linearly, -1 = undefined outside the time range).
    info = np.array([[0, 1, 0, -1]] + [[2, i + 2, 0, -1] for i in range(len(names))],
                    dtype="i4")
    data_1 = np.array([[t[0]], [t[-1]]])               # start/stop row; column 1 is time
    data_2 = np.column_stack(cols)
    aclass = np.array(["Atrajectory", "1.1", "", "binTrans"])
    with open(path, "wb") as fh:
        fh.write(v4_matrix("Aclass", aclass))
        # binTrans: every matrix but Aclass is stored transposed, and read back with .T
        fh.write(v4_matrix("name", np.array(all_names), columns=True))
        fh.write(v4_matrix("description", np.array(texts), columns=True))
        fh.write(v4_matrix("dataInfo", info.T.copy()))
        fh.write(v4_matrix("data_1", data_1.T.copy()))
        fh.write(v4_matrix("data_2", data_2.T.copy()))
    return path
