#!/usr/bin/env python

import subprocess

current_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]).strip().decode('utf-8')
print("Current branch:", current_branch)


ubuntu = f"[![Ubuntu C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/ubuntu.yml?label=Ubuntu&logo=ubuntu&style=flat-square?branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/ubuntu.yml)\n"
macos = f"[![macOS C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/macos.yml?label=macOS&logo=apple&style=flat-square?branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/macos.yml)\n"
windows = f"[![Windows C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/windows.yml?label=Windows&logo=windows&style=flat-square?branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/windows.yml)\n"
codeql = f"[![CodeQL](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/codeql-analysis.yml?label=CodeQL&logo=github&style=flat-square?branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/codeql-analysis.yml)\n"
codecov = f"[![codecov](https://img.shields.io/codecov/c/github/cda-tum/mtct?label=Coverage&logo=codecov&style=flat-square?branch={current_branch})](https://codecov.io/gh/cda-tum/mtct)\n"


lines = open("README.md").readlines()
with open("README.md", "w") as f:
    for line in lines:
        if ubuntu[:20] in line:
            print(f"Replacing {line} with {ubuntu}")
            f.write(ubuntu)
        elif macos[:20] in line:
            print(f"Replacing {line} with {macos}")
            f.write(macos)
        elif windows[:20] in line:
            print(f"Replacing {line} with {windows}")
            f.write(windows)
        elif codeql[:20] in line:
            print(f"Replacing {line} with {codeql}")
            f.write(codeql)
        elif codecov[:20] in line:
            print(f"Replacing {line} with {codecov}")
            f.write(codecov)
        else:
            f.write(line)
