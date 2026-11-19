#!/usr/bin/env python3
# Copyright © 2026 Model Based Innovation LLC.
# SPDX-License-Identifier: BSD-3-Clause -- the text is BSD-3-Clause.txt in the export bundle
# (ModCheck/Resources/Testing/ in ModCheck, Resources/Testing/ in a library carrying a copy).
"""
Export a library's ModCheck reference results -- `ReferenceResults/<case>.npz` plus
`manifest.json`, kept with the library -- to the files other Modelica regression tools read:

    ReferenceResults/<Pkg>/<Sub>/<Model>/<Model>.csv   the Modelica Association's layout (the
    ReferenceResults/<Pkg>/<Sub>/<Model>/<Model>.mat   MSL's ReferenceResults, csv-compare), and
    .../comparisonSignals.txt, .../creation.txt         OpenModelica's LibraryTesting with
                                                         delimiter "/" and extra name $ClassName
    Cases/<case>/<case>.csv|.mat|...                    every other case: one with modifiers
                                                         of a model whose slot is taken
    conf-openmodelica.json                              LibraryTesting config entries, one per library
    README.md, provenance.json, LICENSE                 what it is, who produced it, its terms

The export is produced on demand and is not kept: the stored data stays npz, and the export
is regenerated from it whenever it is wanted. Its data carries the library's own license
(the library's LICENSE file is copied into it).

    python3 export_references.py                     # reads interop.yaml beside this script
    python3 export_references.py path/to/interop.yaml --out /tmp/export
    python3 export_references.py --case MR_Examples_Foo --format csv

interop.yaml needs one key, `config:` (a ModCheck case list, or a list of them); everything
else has a default -- see interop.example.yaml. Needs numpy and PyYAML, and dsres.py beside
this file (a copy of ModCheck's backends/dsres.py). Nothing else from ModCheck.

Exit code = the number of cases that should have been exported and could not be. A case left
out by rule (disabled, no stored reference, not a trajectory, a single point) is listed in
README.md with its reason and does not count.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import re
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import dsres  # noqa: E402  (the copy beside this file)

FORMATS = ("csv", "mat")
MARKER = ".modcheck-export"           # an out directory holding this may be replaced
LICENSE_NAMES = ("LICENSE", "LICENSE.txt", "LICENSE.md", "LICENCE", "COPYING")
DEFAULT_TOLERANCE = 1e-3              # ModCheck's own default for `tolerance:`
CASE_NOTE_KEYS = ("check_mode", "variable_tolerances", "exclude_variables",
                  "skip_initial_fraction", "expect_failure")


# ------------------------------------------------------------------ configuration
def load_settings(path: Path) -> dict:
    s = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if "config" not in s:
        raise SystemExit(f"{path}: `config:` (the ModCheck case list) is required")
    s["_dir"] = path.resolve().parent
    configs = s["config"] if isinstance(s["config"], list) else [s["config"]]
    s["config"] = [(s["_dir"] / c).resolve() for c in configs]
    fmts = s.get("formats", list(FORMATS))
    bad = [f for f in fmts if f not in FORMATS]
    if bad:
        raise SystemExit(f"{path}: unknown format(s) {bad}; choose from {list(FORMATS)}")
    s["formats"] = fmts
    s["out"] = (s["_dir"] / s.get("out", "Export")).resolve()
    s.setdefault("model_case", {})
    return s


def find_license(settings: dict) -> Path | None:
    if settings.get("license"):
        p = (settings["_dir"] / settings["license"]).resolve()
        if not p.is_file():
            raise SystemExit(f"license: {p} does not exist")
        return p
    d = settings["_dir"]
    for d in [d, *d.parents]:
        for n in LICENSE_NAMES:
            if (d / n).is_file():
                return d / n
    return None


def library_version(settings: dict, start: Path | None = None) -> str:
    """`library_version:`, else the `version` of the nearest package.mo above `start`
    (default: interop.yaml's folder)."""
    if settings.get("library_version") and start is None:
        return str(settings["library_version"])
    for d in (start or settings["_dir"]).parents:
        pkg = d / "package.mo"
        if pkg.is_file():
            text = _drop_calls(pkg.read_text(encoding="utf-8", errors="replace"),
                               ("uses", "conversion"))
            m = re.search(r'\bversion\s*=\s*"([^"]*)"', text)
            if m:
                return m.group(1)
    return "main"


def _drop_calls(text: str, names: tuple) -> str:
    """`text` without each `<name>(...)` (balanced parentheses): the versions inside
    uses(Modelica(version="4.1.0")) and conversion(from(version=...)) are not the library's."""
    for name in names:
        while (m := re.search(rf"\b{name}\s*\(", text)):
            depth, i = 0, m.end() - 1
            while i < len(text):
                depth += {"(": 1, ")": -1}.get(text[i], 0)
                if depth == 0:
                    break
                i += 1
            text = text[:m.start()] + text[i + 1:]
    return text


# ------------------------------------------------------------------ reading a suite
def _manifest(refdir: Path) -> dict:
    p = refdir / "manifest.json"
    if not p.is_file():
        return {}
    data = json.loads(p.read_text(encoding="utf-8"))
    entries = data.get("cases", []) if isinstance(data, dict) else data
    return {e["name"]: e for e in entries or []}


def _is_lfs_pointer(path: Path) -> bool:
    with open(path, "rb") as fh:
        return fh.read(40).startswith(b"version https://git-lfs")


def collect(settings: dict, only: set | None) -> tuple[list[dict], list[tuple[str, str]]]:
    """Every case of every case list: (exportable cases, [(case, reason left out)])."""
    cases, left_out, seen = [], [], set()
    for cfg_path in settings["config"]:
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        refdir = (cfg_path.parent / settings.get("references", "ReferenceResults")).resolve()
        manifest = _manifest(refdir)
        for case in cfg.get("cases", []) or []:
            name = case["name"]
            if only and name not in only:
                continue
            if name in seen:
                left_out.append((name, f"repeated in {cfg_path.name}; the first is exported"))
                continue
            seen.add(name)
            if case.get("enabled", True) is False:
                left_out.append((name, "disabled in the case list"))
                continue
            npz = refdir / f"{name}.npz"
            if not npz.is_file():
                why = ("stored as Parquet; this export reads npz"
                       if (refdir / f"{name}.parquet").is_file() else "no stored reference")
                left_out.append((name, why))
                continue
            if _is_lfs_pointer(npz):
                left_out.append((name, "a Git LFS pointer: run `git lfs pull` first"))
                continue
            with np.load(npz, allow_pickle=False) as z:
                arrays = {k: z[k] for k in z.files}
            if not _tabular(arrays) or "time" not in arrays:
                left_out.append((name, "not a trajectory (a linearization's matrices)"))
                continue
            if len(arrays["time"]) < 2:
                left_out.append((name, "a single point (a steady-state result): "
                                       "csv-compare and diffSimulationResults need two"))
                continue
            entry = manifest.get(name, {})
            model = case.get("model") or entry.get("model")
            if not model:
                left_out.append((name, "no model named in the case list or the manifest"))
                continue
            cases.append({"name": name, "model": model, "case": case, "cfg": cfg,
                          "cfg_path": cfg_path, "entry": entry, "arrays": arrays})
    return cases, left_out


def _tabular(arrays: dict) -> bool:
    arrs = [np.asarray(a) for a in arrays.values()]
    return bool(arrs) and all(a.ndim == 1 and a.dtype.kind in "fiub" for a in arrs) \
        and len({len(a) for a in arrs}) == 1


def assign_slots(cases: list[dict], overrides: dict) -> None:
    """Each model's folder in ReferenceResults/ holds one case: the one `model_case:` names,
    else its first case without modifiers, else its first case. The rest go to Cases/."""
    by_model: dict[str, list[dict]] = {}
    for c in cases:
        by_model.setdefault(c["model"], []).append(c)
    for model, group in by_model.items():
        chosen = None
        if model in overrides:
            chosen = next((c for c in group if c["name"] == overrides[model]), None)
            if chosen is None:
                print(f"warning: model_case names {overrides[model]!r} for {model}, "
                      f"which is not an exported case of it")
        chosen = chosen or next((c for c in group if not c["case"].get("modifiers")), group[0])
        for c in group:
            c["slot"] = c is chosen


# ------------------------------------------------------------------ Modelica text
_IDENT = re.compile(r"[^A-Za-z0-9_]")


def modelica_value(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(v)
    if isinstance(v, (list, tuple)):
        return "{" + ", ".join(modelica_value(x) for x in v) + "}"
    if isinstance(v, dict):
        return "(" + modifier_text(v) + ")"
    return str(v)                     # an expression, an enumeration literal, a redeclare


def modifier_text(mods: dict) -> str:
    return ", ".join(f"{k}{modelica_value(v)}" if isinstance(v, dict)
                     else f"{k}={modelica_value(v)}" for k, v in mods.items())


def modified_class(case_name: str, model: str, mods: dict) -> str:
    """The case as the Modelica model it is: `model <Case> = <Model>(<modifiers>);`."""
    ident = _IDENT.sub("_", case_name)
    if ident[:1].isdigit():
        ident = "_" + ident
    return f"model {ident} = {model}({modifier_text(mods)});"


# ------------------------------------------------------------------ writing
def _signals(arrays: dict) -> list[str]:
    return ["time"] + [k for k in arrays if k != "time"]


def write_csv(path: Path, arrays: dict) -> None:
    names = _signals(arrays)
    data = np.column_stack([np.asarray(arrays[n], dtype="f8") for n in names])
    header = ",".join(f'"{n}"' for n in names)
    # %.17g round-trips every float64 exactly, and is what the MSL's own references use
    np.savetxt(path, data, fmt="%.17g", delimiter=",", header=header, comments="",
               encoding="utf-8")


def write_mat(path: Path, arrays: dict, units: dict, time_name: str) -> None:
    desc = {n: f"[{u}]" for n, u in (units or {}).items() if u and u.isascii()}
    dsres.write(path, {n: arrays[n] for n in _signals(arrays)}, time_name=time_name,
                descriptions=desc)


def _settings_of(c: dict) -> dict:
    case, cfg, t = c["case"], c["cfg"], c["arrays"]["time"]
    start = float(case.get("start_time", t[0]) or 0.0)
    stop = float(case.get("final_time", t[-1]))
    ncp = case.get("ncp")
    rtol = (c["entry"].get("metrics") or {}).get("rtol", case.get("rtol"))
    return {"start": start, "stop": stop, "ncp": ncp, "rtol": rtol,
            "interval": (stop - start) / ncp if ncp else None,
            "tolerance": float(case.get("tolerance", cfg.get("tolerance", DEFAULT_TOLERANCE))),
            "custom_function": case.get("custom_function", cfg.get("custom_function", "dynamic"))}


def _producer(entry: dict) -> tuple[str, str, str]:
    """(descriptor, generation tool, UTC time) of the stored reference, from its record."""
    rec = entry.get("record") or {}
    tool = (rec.get("executable") or {}).get("generation_tool") or ""
    utc = (rec.get("run") or {}).get("utc") or ""
    return entry.get("descriptor") or "unknown", tool, utc


def creation_text(c: dict, license_name: str | None) -> str:
    s, (descriptor, tool, utc) = _settings_of(c), _producer(c["entry"])
    rec = c["entry"].get("record") or {}
    lines = ["[ResultCreationLog]", f'modelName="{c["model"]}"', "",
             "// Test info",
             f'generationTool="{descriptor}' + (f", {tool}" if tool else "") + '"']
    if utc:
        lines.append(f'generationDateAndTime="{utc}"')
    lines += [f'exportTool="ModCheck export_references.py"',
              f'modcheckCase="{c["name"]}"',
              f'modcheckCaseList="{c["cfg_path"].name}"']
    if rec.get("record_id"):
        lines.append(f'modcheckRecordId="{rec["record_id"]}"')
    if c["entry"].get("reference_from"):
        lines.append(f'// reference_from: {c["entry"]["reference_from"]} -- this case\'s '
                     f'reference came from another tool than the case list\'s own')
    lines += ["", "// Experiment settings (as ModCheck produced the reference)",
              f"StartTime={s['start']:g}", f"StopTime={s['stop']:g}"]
    if s["interval"]:
        lines.append(f"Interval={s['interval']:g} // (StopTime-StartTime)/{s['ncp']}")
    if s["rtol"] is not None:
        lines.append(f"Tolerance={float(s['rtol']):g}")
    if s["custom_function"] != "dynamic":
        lines.append(f"// ModCheck custom function: {s['custom_function']}")
    mods = c["case"].get("modifiers") or {}
    if mods:
        lines += ["", f"// Modifiers: this reference is of the model below, not of {c['model']} "
                      "as it stands",
                  "// " + modified_class(c["name"], c["model"], mods)]
    lines += ["", "// ModCheck comparison: a tube of tolerance x (max-min) of each reference "
                  "signal, one output interval wide in time",
              f"// tolerance={s['tolerance']:g}"]
    for k in CASE_NOTE_KEYS:
        if k in c["case"]:
            lines.append(f"// {k}: {json.dumps(c['case'][k], sort_keys=True)}")
    if license_name:
        lines += ["", f"// These data are licensed as the library is: see {license_name} at the "
                      "root of this export."]
    return "\n".join(lines) + "\n"


def write_case(c: dict, root: Path, formats: list, time_name: str,
               license_name: str | None) -> Path:
    if c["slot"]:
        folder, stem = root / "ReferenceResults" / Path(*c["model"].split(".")), \
            c["model"].split(".")[-1]
    else:
        folder, stem = root / "Cases" / c["name"], c["name"]
    folder.mkdir(parents=True, exist_ok=True)
    if "csv" in formats:
        write_csv(folder / f"{stem}.csv", c["arrays"])
    if "mat" in formats:
        write_mat(folder / f"{stem}.mat", c["arrays"], c["entry"].get("units") or {}, time_name)
    (folder / "comparisonSignals.txt").write_text(
        "\n".join(_signals(c["arrays"])) + "\n", encoding="utf-8")
    (folder / "creation.txt").write_text(creation_text(c, license_name), encoding="utf-8")
    return folder


def _most_common(values: list):
    """The value most cases share; on a tie, the loosest of the tied ones."""
    counts = {v: values.count(v) for v in values}
    top = max(counts.values())
    return max(v for v, n in counts.items() if n == top)


def om_tolerances(slot_cases: list[dict], overrides: dict) -> tuple[dict, list[str]]:
    """OpenModelica's tube settings for the whole library, and the cases they do not fit.
    ModCheck's tube is tolerance x (max - min) of the reference, one output interval wide
    in time; the nearest omc settings are reference_reltol / reference_reltolDiffMinMax =
    tolerance and reference_rangeDelta = 1/ncp. omc takes one value per library, so each is
    the value most cases use -- not the loosest, which would loosen every other model to
    fit one -- and the cases with another value are listed in README.md."""
    settings = [_settings_of(c) for c in slot_cases]
    tol = _most_common([s["tolerance"] for s in settings] or [DEFAULT_TOLERANCE])
    ncps = [s["ncp"] for s in settings if s["ncp"]]
    out = {"reference_reltol": tol, "reference_reltolDiffMinMax": tol}
    if ncps:
        out["reference_rangeDelta"] = 1.0 / _most_common(ncps)
    out.update({k: v for k, v in (overrides or {}).items() if k.startswith("reference_")})
    differ = []
    for c, s in zip(slot_cases, settings):
        why = []
        if s["tolerance"] != out["reference_reltolDiffMinMax"]:
            why.append(f"tolerance {s['tolerance']:g}")
        if s["ncp"] and "reference_rangeDelta" in out and \
                abs(1.0 / s["ncp"] - out["reference_rangeDelta"]) > 1e-12:
            why.append(f"ncp {s['ncp']} (rangeDelta {1.0 / s['ncp']:g})")
        if c["case"].get("variable_tolerances") or c["case"].get("check_mode"):
            why.append("per-variable tolerance or check mode")
        if why:
            differ.append(f"{c['name']}: " + ", ".join(why))
    return out, differ


def _library_of(c: dict) -> str:
    """The library a case's model belongs to: its case list's `library:`, else the model's
    top-level package."""
    return c["cfg"].get("library") or c["model"].split(".")[0]


def openmodelica_conf(settings: dict, slot_cases: list[dict], cfgs: list[dict]) -> tuple:
    """OpenModelicaLibraryTesting config entries for the ReferenceResults/ tree, one per
    library (case lists of several libraries share one export, e.g. a library and its
    separate test library), and the cases their tolerances do not fit. Their
    getReferenceFileName() builds <referenceFiles>/<Model with . -> delimiter>[.<extra
    name>].<ext>; with "/" and "$ClassName" that is <Pkg>/<..>/<Model>/<Model>.<ext>, so
    every entry points at the same tree. `library:` and `library_version:` in interop.yaml
    name the first case list's library; each other library's version is the `version` of
    the nearest package.mo above its case list."""
    om = settings.get("openmodelica") or {}
    first = next((c.get("library") for c in cfgs[:1] if c.get("library")), None)
    groups: dict[str, list[dict]] = {}
    for c in slot_cases:
        groups.setdefault(_library_of(c), []).append(c)
    if not groups:
        groups[first or "LIBRARY"] = []
    entries, differ = [], []
    for i, (lib, cases) in enumerate(groups.items()):
        own = lib == first or (first is None and i == 0)
        tols, d = om_tolerances(cases, om)
        differ += d
        version = (library_version(settings) if own else
                   library_version(settings, cases[0]["cfg_path"]))
        entries.append({
            "library": (settings.get("library") or lib) if own else lib,
            "libraryVersion": version,
            "referenceFiles": str(settings["out"] / "ReferenceResults"),
            "referenceFileExtension": "mat" if "mat" in settings["formats"] else "csv",
            "referenceFileNameDelimiter": "/",
            "referenceFileNameExtraName": "$ClassName",
            **tols,
        })
    return entries, differ


# ------------------------------------------------------------------ README
def readme_text(settings, cases, left_out, failed, license_path, conf, differ) -> str:
    slot = [c for c in cases if c["slot"] and c["name"] not in failed]
    other = [c for c in cases if not c["slot"] and c["name"] not in failed]
    producers = sorted({_producer(c["entry"])[0] for c in cases})
    now = _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lib = " and ".join(e["library"] for e in conf)
    out = [f"# Reference results for {lib}",
           "",
           f"Exported {now} by `export_references.py` (ModCheck) from "
           + ", ".join(f"`{p.name}`" for p in settings["config"])
           + ". Regenerate it rather than editing it: the stored references are the npz "
             "files beside the case list, and this folder is derived from them.",
           "",
           "## License",
           "",
           (f"These data are part of {lib} and are licensed as the library is: see "
            f"[`{license_path.name}`]({license_path.name}), a copy of the library's own."
            if license_path else
            "**No LICENSE file was found above interop.yaml.** Set `license:` in "
            "interop.yaml before passing this export on."),
           "",
           "## Produced by",
           ""]
    out += [f"* {p}" for p in producers]
    out += ["",
            "The tool that produced each reference is in its `creation.txt` "
            "(`generationTool`) and in `provenance.json`. A reference produced by one tool "
            "and checked with another tests the tools against each other as much as the "
            "library.",
            "",
            "## Layout",
            "",
            "| path | for |",
            "|---|---|",
            "| `ReferenceResults/<Pkg>/<..>/<Model>/<Model>.csv` | csv-compare (`--mode "
            "csvTreeCompare -d \",\"`), the MSL's layout, pandas |",
            "| `ReferenceResults/<Pkg>/<..>/<Model>/<Model>.mat` | OpenModelica's "
            "`diffSimulationResults`, Dymola, OMEdit, MATLAB, `scipy.io.loadmat` |",
            "| `.../comparisonSignals.txt` | the signals compared, `time` first: "
            "OpenModelica's LibraryTesting reads it beside the reference |",
            "| `.../creation.txt` | the producing tool, the experiment settings, modifiers "
            "and ModCheck's tolerance, in the MSL's `[ResultCreationLog]` format |",
            "| `Cases/<case>/` | the same four files for a case that is not its model's "
            "entry above: a second case of one model, with modifiers |",
            "| `conf-openmodelica.json` | an entry per library for "
            "OpenModelicaLibraryTesting's configs, all pointing at `ReferenceResults/` |",
            "| `provenance.json` | per case: model, modifiers, settings, producer, record id |",
            "",
            "`time` is the first column; an event appears as two rows with the same time. "
            "Values are written with 17 significant digits, which reproduces the stored "
            "float64 exactly.",
            "",
            "## Using it",
            "",
            "```",
            "# csv-compare: a result tree with the same layout against this one",
            "Compare --mode csvTreeCompare -d \",\" --tolerance 0.002 --reportdir report "
            "<your-results> ReferenceResults",
            "",
            "# OpenModelica LibraryTesting: put conf-openmodelica.json's entries into a config,",
            "# adjust referenceFiles if this folder moved, and run test.py on it",
            "",
            "# Python",
            "import pandas as pd; df = pd.read_csv('ReferenceResults/.../Model.csv')",
            "```",
            "",
            "## What a pass means elsewhere",
            "",
            "ModCheck compares each signal inside a tube of `tolerance` x (max - min) of the "
            "reference, one output interval wide in time, and re-simulates at 100x the "
            "reference's solver tolerance. `conf-openmodelica.json` sets the nearest "
            "OpenModelica values (`reference_reltolDiffMinMax`, `reference_reltol`, "
            "`reference_rangeDelta` = 1/ncp), one set per library, at the value most of its "
            "cases use "
            "(override them under `openmodelica:` in interop.yaml). "
            "csv-compare and `diffSimulationResults` define their tubes themselves, so their "
            "verdicts need not agree with ModCheck's. Both also simulate with the model's own "
            "`experiment` annotation, while the settings below are the ones the reference "
            "was produced with.",
            "",
            "A case with modifiers is a different model from the one its folder is named "
            "after. Its `creation.txt` states that model as a Modelica class in a comment "
            "line, e.g. `// model Case = Pkg.Model(k=2);`. A tool that simulates the named "
            "model without the modifiers is not expected to reproduce it.",
            "",
            *([f"OpenModelica's per-library settings do not fit {len(differ)} case(s); "
               "they may pass or fail there for that reason alone:", ""]
              + [f"* {d}" for d in differ] + [""] if differ else []),
            f"## Cases ({len(slot)} in ReferenceResults/, {len(other)} in Cases/)",
            "",
            "| case | model | where | start | stop | interval | rtol | tolerance | modifiers |",
            "|---|---|---|---|---|---|---|---|---|"]
    for c in slot + other:
        s = _settings_of(c)
        mods = modifier_text(c["case"].get("modifiers") or {})
        out.append(f"| {c['name']} | `{c['model']}` | {'model' if c['slot'] else 'Cases/'} | "
                   f"{s['start']:g} | {s['stop']:g} | "
                   f"{'' if s['interval'] is None else format(s['interval'], 'g')} | "
                   f"{'' if s['rtol'] is None else format(float(s['rtol']), 'g')} | "
                   f"{s['tolerance']:g} | {('`' + mods + '`') if mods else ''} |")
    if left_out or failed:
        out += ["", "## Not exported", "", "| case | reason |", "|---|---|"]
        out += [f"| {n} | {why} |" for n, why in left_out]
        out += [f"| {n} | ERROR: {why} |" for n, why in failed.items()]
    return "\n".join(out) + "\n"


# ------------------------------------------------------------------ main
def prepare_out(out: Path) -> None:
    if out.exists():
        if not (out / MARKER).is_file() and any(out.iterdir()):
            raise SystemExit(f"{out} exists, is not empty and is not an export "
                             f"(no {MARKER}); refusing to replace it")
        shutil.rmtree(out)
    out.mkdir(parents=True)
    (out / MARKER).write_text("written by export_references.py; replaced on every export\n",
                              encoding="utf-8")


def export(settings: dict, only: set | None = None) -> int:
    cases, left_out = collect(settings, only)
    assign_slots(cases, settings["model_case"])
    out = settings["out"]
    prepare_out(out)
    license_path = find_license(settings)
    if license_path:
        shutil.copyfile(license_path, out / license_path.name)
    lic = license_path.name if license_path else None
    time_name = settings.get("mat", {}).get("time_name", "time") \
        if isinstance(settings.get("mat"), dict) else "time"
    failed: dict[str, str] = {}
    for c in cases:
        try:
            write_case(c, out, settings["formats"], time_name, lic)
        except Exception as e:           # one bad case must not stop the export
            failed[c["name"]] = f"{type(e).__name__}: {e}"
            print(f"FAIL {c['name']}: {failed[c['name']]}")
    cfgs = [yaml.safe_load(p.read_text(encoding="utf-8")) or {} for p in settings["config"]]
    slot_ok = [c for c in cases if c["slot"] and c["name"] not in failed]
    conf, differ = openmodelica_conf(settings, slot_ok, cfgs)
    (out / "conf-openmodelica.json").write_text(json.dumps(conf, indent=2) + "\n",
                                                 encoding="utf-8")
    prov = {"license": lic, "cases": []}
    for c in cases:
        if c["name"] in failed:
            continue
        descriptor, tool, utc = _producer(c["entry"])
        prov["cases"].append({
            "case": c["name"], "model": c["model"],
            "path": (f"ReferenceResults/{'/'.join(c['model'].split('.'))}" if c["slot"]
                     else f"Cases/{c['name']}"),
            "modifiers": c["case"].get("modifiers") or {},
            "modelica": (modified_class(c["name"], c["model"], c["case"]["modifiers"])
                         if c["case"].get("modifiers") else None),
            "settings": _settings_of(c), "signals": _signals(c["arrays"]),
            "units": c["entry"].get("units") or {},
            "produced_by": descriptor, "generation_tool": tool, "utc": utc,
            "reference_from": c["entry"].get("reference_from"),
            "record_id": (c["entry"].get("record") or {}).get("record_id"),
            "case_list": c["cfg_path"].name})
    (out / "provenance.json").write_text(json.dumps(prov, indent=1) + "\n", encoding="utf-8")
    (out / "README.md").write_text(
        readme_text(settings, cases, left_out, failed, license_path, conf, differ),
        encoding="utf-8")
    n_slot = sum(1 for c in slot_ok)
    print(f"exported {len(cases) - len(failed)} case(s) to {out}: {n_slot} in "
          f"ReferenceResults/, {len(cases) - len(failed) - n_slot} in Cases/; "
          f"{len(left_out)} left out, {len(failed)} failed")
    if not license_path:
        print("warning: no LICENSE found above interop.yaml; set `license:`")
    return len(failed)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("settings", nargs="?", default=str(HERE / "interop.yaml"),
                    help="interop.yaml (default: the one beside this script)")
    ap.add_argument("--out", help="export directory (default: `out:`, else Export/ beside "
                                  "interop.yaml); replaced if it holds an earlier export")
    ap.add_argument("--case", action="append", help="export only this case (repeatable)")
    ap.add_argument("--format", nargs="+", choices=FORMATS, help="overrides `formats:`")
    a = ap.parse_args(argv)
    settings = load_settings(Path(a.settings))
    if a.out:
        settings["out"] = Path(a.out).resolve()
    if a.format:
        settings["formats"] = a.format
    return export(settings, set(a.case) if a.case else None)


if __name__ == "__main__":
    sys.exit(main())
