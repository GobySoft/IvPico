#!/usr/bin/env python3
"""Validates .bhv.yml files against IvPico's JSON Schema.

Usage: validate_schema.py <bhv.schema.json> <ivpico_bhv> <fixtures dir> [file.bhv.yml ...]

Every fixture .bhv is imported with ivpico_bhv and must validate, as must each
file given. A few files that ivpico_bhv's reader rejects must fail validation
too, so a schema that accepts anything can't pass.
"""

import pathlib
import subprocess
import sys
import tempfile

import jsonschema
import yaml

REJECTED = {
    "unknown top-level key": "behavior:\n  - type: A\n",
    "behavior without a type": "behaviors:\n  - name: a\n",
    "unknown behavior key": "behaviors:\n  - type: A\n    endflags: X = 1\n",
    "list for a single value": "behaviors:\n  - type: A\n    pwt: [1, 2]\n",
    "map for a parameter": "behaviors:\n  - type: A\n    params:\n      speed: {a: 1}\n",
    "mode without set": "modes:\n  - condition: A = 1\n",
    "initialize with a list": "initialize:\n  A: [1, 2]\n",
}


def main():
    schema_file, tool, fixtures, *files = sys.argv[1:]
    validator = jsonschema.Draft202012Validator(
        yaml.safe_load(pathlib.Path(schema_file).read_text()))
    failures = []

    def validate(name, text):
        errors = list(validator.iter_errors(yaml.safe_load(text)))
        for e in errors:
            failures.append(f"{name}: {'/'.join(map(str, e.path))}: {e.message}")
        return not errors

    with tempfile.TemporaryDirectory() as tmp:
        bhvs = sorted(pathlib.Path(fixtures).glob("*/*.bhv"))
        for bhv in bhvs:
            out = pathlib.Path(tmp) / f"{bhv.parent.name}.{bhv.name}.yml"
            subprocess.run([tool, "import", str(bhv), "-o", str(out)], check=True)
            validate(str(bhv), out.read_text())
        if len(bhvs) < 10:
            failures.append(f"only {len(bhvs)} fixture .bhv files in {fixtures}")

    for f in files:
        validate(f, pathlib.Path(f).read_text())

    for what, text in REJECTED.items():
        if not list(validator.iter_errors(yaml.safe_load(text))):
            failures.append(f"the schema accepts a file with a {what}")

    for f in failures:
        print(f)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
