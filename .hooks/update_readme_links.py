#!/usr/bin/env python

import subprocess

try:
    current_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]).strip()
    print("Current branch:", current_branch)
except Exception as e:
    print("Error:", e)


ubuntu = f"[![Ubuntu C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/ubuntu.yml?label=Ubuntu&logo=ubuntu&style=flat-square?branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/ubuntu.yml)".format(current_branch=current_branch)

lines = open("README.md").readlines()
