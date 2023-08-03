#!/usr/bin/env python

import subprocess

current_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]).strip().decode('utf-8')
print("Current branch:", current_branch)


ubuntu = f"[![Ubuntu C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/ubuntu.yml?label=Ubuntu&logo=ubuntu&style=flat-square?branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/ubuntu.yml)\n"

lines = open("README.md").readlines()
for line in lines:
    if ubuntu[:20] in line:
        print(f"Replacing {line} with {ubuntu}")
