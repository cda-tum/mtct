#!/usr/bin/env python

import subprocess

current_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]).strip().decode('utf-8')
print("Current branch:", current_branch)


ubuntu = f"[![Ubuntu C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/ubuntu.yml?label=Ubuntu&logo=ubuntu&style=flat-square&branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/ubuntu.yml?query=branch%3A{current_branch})\n"
macos = f"[![macOS C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/macos.yml?label=macOS&logo=apple&style=flat-square&branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/macos.yml?query=branch%3A{current_branch})\n"
windows = f"[![Windows C++](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/windows.yml?label=Windows&logo=windows&style=flat-square&branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/windows.yml?query=branch%3A{current_branch})\n"
codeql = f"[![CodeQL](https://img.shields.io/github/actions/workflow/status/cda-tum/mtct/codeql-analysis.yml?label=CodeQL&logo=github&style=flat-square&branch={current_branch})](https://github.com/cda-tum/mtct/actions/workflows/codeql-analysis.yml?query=branch%3A{current_branch})\n"
codecov = f"[![codecov](https://img.shields.io/codecov/c/github/cda-tum/mtct?label=Coverage&logo=codecov&style=flat-square&branch={current_branch})](https://codecov.io/gh/cda-tum/mtct/tree/{current_branch})\n"
lic = f"[![License](https://img.shields.io/github/license/cda-tum/mtct?label=License&style=flat-square&branch={current_branch})](https://github.com/cda-tum/mtct/blob/{current_branch}/LICENSE)\n"


lines = open("README.md").readlines()
with open("README.md", "w") as f:
    for line in lines:
        if ubuntu[:20] in line and ubuntu not in line:
            print(f"Replacing {line} with {ubuntu}")
            f.write(ubuntu)
        elif macos[:20] in line and macos not in line:
            print(f"Replacing {line} with {macos}")
            f.write(macos)
        elif windows[:20] in line and windows not in line:
            print(f"Replacing {line} with {windows}")
            f.write(windows)
        elif codeql[:20] in line and codeql not in line:
            print(f"Replacing {line} with {codeql}")
            f.write(codeql)
        elif codecov[:20] in line and codecov not in line:
            print(f"Replacing {line} with {codecov}")
            f.write(codecov)
        elif lic[:20] in line and lic not in line:
            print(f"Replacing {line} with {lic}")
            f.write(lic)
        else:
            f.write(line)
