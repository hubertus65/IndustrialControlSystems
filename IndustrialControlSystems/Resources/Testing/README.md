# Reference results for other regression tools

This folder turns the library's regression references into the files other Modelica
regression tools read. Those tools are the Modelica Association's csv-compare (the MSL's
`ReferenceResults` layout) and OpenModelica's LibraryTesting. It needs Python 3 with numpy
and PyYAML. It does not need ModCheck or any simulation tool.

The references themselves are the `ReferenceResults/<case>.npz` files beside the
library's ModCheck case list, with their `manifest.json`. They stay in that form. The
export is produced from them when it is wanted, is not committed, and is replaced every
time it is run.

## Run it

```
python3 export_references.py                        # reads interop.yaml in this folder
python3 export_references.py interop.yaml --out /tmp/refs
python3 export_references.py --case <case name> --format csv
```

`interop.yaml` needs one line, the case list:

```yaml
config: ../ModCheck/regression_cases.yaml
```

The other settings are optional; `interop.example.yaml` lists them with their defaults.
The export goes to `Export/` in this folder unless `out:` says otherwise. Add `Export/` to
`.gitignore`.

## What you get

```
Export/
  ReferenceResults/<Pkg>/<..>/<Model>/<Model>.csv     time first, quoted header, 17 digits
                                   .../<Model>.mat     MAT v4 (dsres), as Dymola and OpenModelica write
                                   .../comparisonSignals.txt   the signals to compare
                                   .../creation.txt            which tool produced it, the settings
  Cases/<case>/...                    cases that are not their model's entry (see Modifiers)
  conf-openmodelica.json              an OpenModelicaLibraryTesting config entry
  README.md                           every case, its settings, what was left out and why
  provenance.json                     the same, machine-readable
  LICENSE                             the library's own license, which the data carry
```

* **csv-compare:** `Compare --mode csvTreeCompare -d "," --tolerance 0.002 --reportdir
  report <your results> Export/ReferenceResults`. csv-compare's default delimiter is `;`,
  so `-d ","` is needed.
* **OpenModelica LibraryTesting:** take the entry in `conf-openmodelica.json` into a config
  and run `test.py` on it. `referenceFiles` is the absolute path of this export; change it
  if the export moves.
* **Anything else:** the CSV reads with `pandas.read_csv`, and the MAT with
  `scipy.io.loadmat`, MATLAB, Dymola or OMEdit, or with `dsres.read()` from this folder.

## Modifiers

ModCheck stores a result per *case*: a model, perhaps with modifiers. The other tools store
one per *model*. A model with modifiers is a different model, so its reference is not the
plain model's reference. Each model's folder holds one case: its first case without
modifiers, or the one `model_case:` names. A case that has modifiers says so in its
`creation.txt`, as a Modelica class in a comment line:

```
// model Case = Pkg.Model(k=2, sub(x=1.5));
```

Every other case goes to `Cases/<case>/`.

## What a pass means

ModCheck checks each signal in a tube of `tolerance` x (max - min) of the reference, one
output interval wide in time. `conf-openmodelica.json` sets OpenModelica's nearest
settings, one value for the whole library: the tolerance and output grid most cases use.
The export's README lists the cases with a different value. csv-compare and OpenModelica
define their tubes in their own way, and they simulate with each model's `experiment`
annotation. A result can therefore pass in ModCheck and fail there, or the other way
round. `creation.txt` records how each reference was produced, to trace such a
difference.

## Left out

A disabled case is left out. So is a case with no stored npz, one that is a Git LFS
pointer (run `git lfs pull`), a linearization (matrices, not trajectories), and a
single-point steady-state result. The export's README names each one and gives the
reason.

## Files

| file | |
|---|---|
| `export_references.py` | the exporter |
| `dsres.py` | reads and writes MAT v4 result files; a copy of ModCheck's `backends/dsres.py` |
| `interop.example.yaml` | the settings, with their defaults |
| `BSD-3-Clause.txt` | the license of these files |

To update this folder, copy ModCheck's `ModCheck/Resources/Testing/` over it again,
keeping `interop.yaml`.

## License

Copyright © 2026 Model Based Innovation LLC. The files of this folder
(`export_references.py`, `dsres.py`, `interop.example.yaml` and this README) are licensed
under the BSD 3-Clause License; the text is [`BSD-3-Clause.txt`](BSD-3-Clause.txt). The
data an export produces are not covered by it: they are the library's, and carry the
library's license.
