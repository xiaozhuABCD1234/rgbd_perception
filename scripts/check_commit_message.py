#!/usr/bin/env python3
"""Validate one Git commit message using Conventional Commits 1.0."""

import argparse
import re
import sys
from pathlib import Path

TYPES = "feat|fix|docs|style|refactor|perf|test|build|ci|chore|revert"
SUBJECT = re.compile(rf"^(?:{TYPES})(?:\([a-z0-9][a-z0-9._/-]*\))?!?: .+$")


def validate(message: str) -> list[str]:
    lines = message.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    subject = lines[0] if lines else ""
    if subject.startswith("Merge ") or subject.startswith('Revert "'):
        return []

    errors: list[str] = []
    if not SUBJECT.fullmatch(subject):
        errors.append(
            "subject must match: <type>(<optional scope>): <summary>; "
            f"allowed types: {TYPES.replace('|', ', ')}"
        )
    if len(subject) > 72:
        errors.append(f"subject is {len(subject)} characters; maximum is 72")
    if subject != subject.strip():
        errors.append("subject must not have leading/trailing whitespace")
    if len(lines) > 1 and lines[1].strip():
        errors.append("subject and body must be separated by a blank line")

    for number, line in enumerate(lines, start=1):
        if line != line.rstrip():
            errors.append(f"line {number} has trailing whitespace")
        if number > 2 and len(line) > 100 and "http://" not in line and "https://" not in line:
            errors.append(f"body line {number} exceeds 100 characters")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("file", nargs="?", type=Path, help="commit message file")
    source.add_argument("--message", help="literal message for local validation")
    args = parser.parse_args()

    message = args.message if args.message is not None else args.file.read_text(encoding="utf-8")
    errors = validate(message)
    if not errors:
        return 0

    print("Invalid commit message:", file=sys.stderr)
    for error in errors:
        print(f"  - {error}", file=sys.stderr)
    print("Example: feat(camera): add Gemini 305 capture adapter", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
