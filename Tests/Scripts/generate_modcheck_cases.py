#!/usr/bin/env python3
"""Generate the ModCheck regression case lists for IndustrialControlSystems and Tests.

Writes two case lists, one per library, each next to the ReferenceResults/ folder
ModCheck stores that library's baselines in:

    IndustrialControlSystems/Resources/regression_cases.yaml   (Applications examples)
    Tests/Resources/regression_cases.yaml                      (every simulatable test)

ModCheck finds ReferenceResults/ beside --config, so the baselines land in
<Library>/Resources/ReferenceResults/.

Signal selection (parsimonious, scaled to what the model tests):
  * one signal per IndustrialControlSystems component instance in the model, its
    primary output (CS for controllers, Q for timers, CV for the counter, y else),
    controllers first;
  * plus `passed` when the model is self-checking (declares or inherits it);
  * capped at MAX_COMPONENT_SIGNALS component signals; a model with more component
    instances keeps the first ones in declaration order.
An MC/DC variant therefore records dut.<output> + passed (2 signals); a multi-controller
example records one output per controller. Models with no library instance (function
checks) record `passed` only.

Time grid from the model's own experiment() annotation: ncp = (Stop - Start) / Interval
when an Interval is given, else 500 (capped at MAX_NCP); rtol two decades below the
annotation Tolerance (default 1e-6), because ModCheck's checking run loosens it 100x.

Pure text, no Modelica tool. Re-run after adding tests:
    python3 Tests/Scripts/generate_modcheck_cases.py
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # repository root
WORKSPACE = "industrialcontrol"
MAX_COMPONENT_SIGNALS = 6
MAX_NCP = 5000
DEFAULT_NCP = 500

# Primary output per library class, by class-path prefix (first match wins).
OUTPUT_BY_PREFIX = [
    ("IndustrialControlSystems.Controllers.Blocks.", "y"),
    ("IndustrialControlSystems.Controllers.Logic.", "y"),
    ("IndustrialControlSystems.Controllers.", "CS"),
    ("IndustrialControlSystems.Logical.Timers.", "Q"),
    # gates are BooleanOutput y[nOutput] (y1, y2 for the 2x2 ones): a glob, since a result
    # filter "x.y" does not match "x.y[1]" and "[1]" in a glob is a character class
    ("IndustrialControlSystems.Logical.LogicalOperations.", "y*"),
    ("IndustrialControlSystems.Logical.Counter.", "CV"),
    ("IndustrialControlSystems.", "y"),
]
SKIP_CLASS_PREFIXES = (
    "IndustrialControlSystems.Icons.",
    "IndustrialControlSystems.LinearSystems.Discrete.Types.",
    "IndustrialControlSystems.Controllers.Interfaces.",
)

# Hand-measured per-case settings, keyed by case name. Each was triaged with the skill's
# procedure: passes bit-identically at --rtol-factor 1, fails under the checking spread,
# and a tighter reference rtol (1e-10) did NOT converge it -- so the comparison is changed,
# not the solver tolerance. Keys: "extra" = YAML lines added to the case, "reason" = comment.
OVERRIDES = {
    "Applications_ProcessControl_LevelControl": {
        "reason": "sampled PI (Ts = 5) in a limit cycle at CSmin: a tiny solver difference shifts the "
                  "0.1-high pulse train by exactly one sample (same pulses, different phase), so CS is "
                  "checked by its signed integral (total valve opening), which the shift leaves unchanged",
        "extra": ["integral_signed_variables: [PI_Level.CS]",
                  "integral_tolerance: 3.0e-3   # band 4.9 on int|CS|dt = 1637; measured 1.01 (~2 pulses)"]},
    "Applications_ControlProblems_ATProcessControl": {
        "reason": "relay-autotuned R_iD settles at CS ~ -1.2e-3 while its range is 3.3: the default "
                  "endpoint band (1e-3 x |endpoint|) is ~1e-6 wide; 5e-2 is 6e-5 absolute",
        "extra": ['variable_tolerances: {"R_iD.CS": 5.0e-2}']},
}

# Steady-state endpoint near zero on a range of ~1: the default band 1e-3 x |endpoint| is
# 1e-7..6e-6 wide, below the checking run's solver noise (errors measured at 100x:
# 1.3e-6..2.8e-5, all exactly 0 at --rtol-factor 1). 5e-2 x |endpoint| is <= 3e-4 absolute.
for _case, _var, _end in [
    ("Controllers_MCDC_Blocks_ComplexPoles_TrackingAntiWindup", "dut.y", "6.1e-3"),
    ("Controllers_MCDC_Blocks_LeadLAG_AntiWindup", "dut.y", "1.4e-4"),
    ("Controllers_MCDC_Blocks_LeadLAG_TrackingAntiWindup", "dut.y", "1.4e-4"),
    ("Controllers_TestI_bias", "processNoCtrl.y", "-1.7e-3"),
]:
    OVERRIDES[_case] = {
        "reason": f"{_var} settles at {_end} on a range of ~1: endpoint band widened for this variable",
        "extra": [f'variable_tolerances: {{"{_var}": 5.0e-2}}']}

# Relay autotuners: the relay switching instants move with the solver tolerance, so dut.CS
# misses the funnel at the 100x spread (excess 1.5e-4..1.3e-3) but passes at 10x. The
# reference is taken at 1e-9 so that the checking run lands on 1e-7.
for _case in [
    "Controllers_MCDC_AutoTuning_ATPIrelayNCdigital_AntiWindup",
    "Controllers_MCDC_AutoTuning_ATPIrelayNCdigital_Base",
    "Controllers_MCDC_AutoTuning_ATPIrelayNCdigital_Bias",
    "Controllers_MCDC_AutoTuning_ATPIrelayNCdigital_Tracking",
    "Controllers_MCDC_AutoTuning_ATPIrelayNCdigital_TrackingAntiWindup",
    "Controllers_MCDC_AutoTuning_ATPIrelayNCdigital_TrackingBias",
    "Controllers_MCDC_AutoTuning_ATPIrelayNCmixedMode_AntiWindup",
    "Controllers_MCDC_AutoTuning_ATPIrelayNCmixedMode_TrackingAntiWindup",
]:
    OVERRIDES[_case] = {"rtol": 1e-9, "reason":
        "relay switching instants move with rtol; reference at 1e-9 so the 100x check runs at 1e-7"}

LIBRARY_SUBPACKAGES = {"Controllers", "LinearSystems", "Logical", "MathOperations"}

EXPERIMENT_RE = re.compile(r"experiment\s*\(([^()]*)\)", re.S)
CLASS_RE = re.compile(r"^\s*(?:partial\s+)?(?:model|block)\s+(\w+)", re.M)
WITHIN_RE = re.compile(r"^\s*within\s+([\w.]+)\s*;", re.M)
EXTENDS_RE = re.compile(r"^\s*extends\s+([\w.]+)", re.M)
IMPORT_RE = re.compile(r"^\s*import\s+(\w+)\s*=\s*([\w.]+)\s*;", re.M)
# component declaration: <Type> <name> [ ( | "comment" | ; | annotation ]
DECL_RE = re.compile(r"^\s*(?:(?:inner|outer|final|replaceable)\s+)*([A-Za-z_][\w.]*)\s+([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*[(\";]|"
                     r"^\s*(?:(?:inner|outer|final|replaceable)\s+)*([A-Za-z_][\w.]*)\s+([A-Za-z_]\w*)\s*$", re.M)
PASSED_RE = re.compile(r"^\s*Boolean\s+passed\b", re.M)


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def strip_strings(text):
    return re.sub(r'"(?:[^"\\]|\\.)*"', '""', text, flags=re.S)


def parse_file(path):
    raw = path.read_text(encoding="utf-8", errors="replace")
    code = strip_strings(strip_comments(raw))
    within = WITHIN_RE.search(code)
    cls = CLASS_RE.search(code)
    if not cls:
        return None
    pkg = within.group(1) if within else ""
    exp = EXPERIMENT_RE.search(strip_comments(raw))
    exp_args = {}
    if exp:
        for k, v in re.findall(r"(\w+)\s*=\s*([-+0-9.eE]+)", exp.group(1)):
            exp_args[k] = float(v)
    imports = dict(IMPORT_RE.findall(code))
    # body before 'equation'/'algorithm'/'annotation(' of the class: declarations
    body = re.split(r"^\s*(?:equation|algorithm|initial\s+equation)\b", code, maxsplit=1, flags=re.M)[0]
    comps = []
    for m in DECL_RE.finditer(body):
        typ, name = (m.group(1), m.group(2)) if m.group(1) else (m.group(3), m.group(4))
        head = typ.split(".")[0]
        if head in imports:
            typ = imports[head] + typ[len(head):]
        elif head in LIBRARY_SUBPACKAGES and pkg.split(".")[0] == "IndustrialControlSystems":
            typ = "IndustrialControlSystems." + typ     # relative name inside the library
        comps.append((typ, name))
    return {
        "class": f"{pkg}.{cls.group(1)}" if pkg else cls.group(1),
        "pkg": pkg,
        "partial": bool(re.search(r"^\s*partial\s+(model|block)", code, re.M)),
        "is_block": bool(re.search(r"^\s*(partial\s+)?block\s", code, re.M)),
        "experiment": exp_args if exp else None,
        "extends": EXTENDS_RE.findall(body),
        "components": comps,
        "passed": bool(PASSED_RE.search(code)),
    }


def output_for(typ):
    if not typ.startswith("IndustrialControlSystems.") or typ.startswith(SKIP_CLASS_PREFIXES):
        return None
    if ".Functions." in typ or ".Types." in typ:
        return None
    for prefix, out in OUTPUT_BY_PREFIX:
        if typ.startswith(prefix):
            return out
    return None


def resolve(info, index, seen=None):
    """Components and `passed` including those inherited from local (non-MSL) bases."""
    seen = seen or set()
    comps, passed = list(info["components"]), info["passed"]
    for base in info["extends"]:
        if base.startswith("Modelica."):
            continue
        cands = [base] if "." in base and base in index else []
        if not cands:   # relative name: look upward from the package
            parts = info["pkg"].split(".")
            for i in range(len(parts), 0, -1):
                c = ".".join(parts[:i] + [base])
                if c in index:
                    cands = [c]
                    break
        for c in cands:
            if c in seen:
                continue
            seen.add(c)
            bc, bp = resolve(index[c], index, seen)
            comps = bc + comps
            passed = passed or bp
    return comps, passed


def ncp_of(exp):
    start, stop = exp.get("StartTime", 0.0), exp.get("StopTime", 1.0)
    if "Interval" in exp and exp["Interval"] > 0:
        n = int(round((stop - start) / exp["Interval"]))
    elif "NumberOfIntervals" in exp:
        n = int(exp["NumberOfIntervals"])
    else:
        n = DEFAULT_NCP
    return max(2, min(n, MAX_NCP))


def fmt(x):
    s = repr(float(x))
    return s if ("e" in s or "." in s) else s + ".0"


def case_name(cls, lib):
    return cls[len(lib) + 1:].replace(".", "_") if cls.startswith(lib + ".") else cls.replace(".", "_")


def build(lib, scan_dirs, header):
    index = {}
    for d in scan_dirs:
        for f in sorted(d.rglob("*.mo")):
            if f.name == "package.mo":
                continue
            info = parse_file(f)
            if info:
                index[info["class"]] = info
    cases = []
    for cls, info in sorted(index.items()):
        if info["experiment"] is None or info["partial"] or info["is_block"]:
            continue
        comps, passed = resolve(info, index)
        sig, seen = [], set()
        for typ, name in comps:
            out = output_for(typ)
            if out and name not in seen:
                seen.add(name)
                sig.append(f"{name}.{out}")
        # controllers first: in a closed loop they are what is under test, the
        # library's linear systems there mostly play the process
        sig.sort(key=lambda v: 0 if v.endswith(".CS") else 1)
        n_comp = len(sig)
        sig = sig[:MAX_COMPONENT_SIGNALS]
        variables = ["time"] + sig + (["passed"] if passed else [])
        exp = info["experiment"]
        tol = exp.get("Tolerance", 1e-6)
        cases.append({
            "name": case_name(cls, lib), "model": cls,
            "start": exp.get("StartTime", 0.0), "stop": exp.get("StopTime", 1.0),
            "ncp": ncp_of(exp), "rtol": tol / 100.0, "variables": variables,
            "n_comp": n_comp, "passed": passed, "reason": None,
        })
        ov = OVERRIDES.get(cases[-1]["name"])
        if ov:
            cases[-1]["rtol"] = ov.get("rtol", cases[-1]["rtol"])
            cases[-1]["reason"] = ov["reason"]
            cases[-1]["extra"] = ov.get("extra", [])
    lines = [header.rstrip(), f"library: {lib}", f"workspace: {WORKSPACE}",
             "custom_function: dynamic", "tolerance: 1.0e-3", "timeout_seconds: 600", "", "cases:"]
    for c in cases:
        note = f"{c['n_comp']} library component(s)"
        if c["n_comp"] > MAX_COMPONENT_SIGNALS:
            note += f", first {MAX_COMPONENT_SIGNALS} recorded"
        if c["passed"]:
            note += "; self-checking (passed)"
        lines += [
            f"  - name: {c['name']}",
            f"    # {note}",
            *([f"    # hand-set: {c['reason']}"] if c["reason"] else []),
            "    enabled: true",
            f"    model: {c['model']}",
            f"    start_time: {fmt(c['start'])}",
            f"    final_time: {fmt(c['stop'])}",
            f"    ncp: {c['ncp']}",
            f"    rtol: {c['rtol']:.1e}".replace("e-0", "e-"),
            "    modifiers: {}",
            "    variables: [" + ", ".join(c["variables"]) + "]",
            *[f"    {e}" for e in c.get("extra", [])],
            "",
        ]
    return "\n".join(lines), cases


HEADER = """# ModCheck regression cases for {lib}.
# Generated by Tests/Scripts/generate_modcheck_cases.py -- edit the script, not this file,
# unless a case needs a hand-tuned override (then note it in the case comment).
# References: ReferenceResults/ beside this file (ModCheck's --ref-dir default).
#   store:  run_regression.sh --config <this file> --store
#   check:  run_regression.sh --config <this file>
# rtol is the model's experiment Tolerance / 100 (the checking run loosens it 100x)."""


def main():
    out = {}
    for lib, dirs, target in [
        ("IndustrialControlSystems", [ROOT / "IndustrialControlSystems" / "Applications"],
         ROOT / "IndustrialControlSystems" / "Resources" / "regression_cases.yaml"),
        ("Tests", [ROOT / "Tests"], ROOT / "Tests" / "Resources" / "regression_cases.yaml"),
    ]:
        text, cases = build(lib, dirs, HEADER.format(lib=lib))
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8")
        nsig = sum(len(c["variables"]) - 1 for c in cases)
        print(f"{lib}: {len(cases)} cases, {nsig} signals -> {target.relative_to(ROOT)}")
        out[lib] = cases
    empty = [c["model"] for cs in out.values() for c in cs if len(c["variables"]) == 1]
    if empty:
        print("WARNING: no signal found for:", *empty, sep="\n  ")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
