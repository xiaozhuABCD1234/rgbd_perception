#!/usr/bin/env python3

import argparse
import subprocess
import sys

from check_commit_message import validate


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], check=True, text=True, stdout=subprocess.PIPE
    ).stdout


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("range", nargs="?", default="HEAD")
    args = parser.parse_args()

    commits = git("log", "--format=%H", "--no-merges", args.range).splitlines()
    if not commits:
        print(f"No commits to check in {args.range}")
        return 0

    failed = 0
    for commit in commits:
        message = git("show", "-s", "--format=%B", commit)
        errors = validate(message)
        if errors:
            failed += 1
            print(f"Invalid commit {commit[:12]}:", file=sys.stderr)
            for error in errors:
                print(f"  - {error}", file=sys.stderr)

    if failed:
        print(f"{failed}/{len(commits)} commits failed validation", file=sys.stderr)
        return 1
    print(f"Validated {len(commits)} commit(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
