#!/bin/bash
git diff -U0 --no-color HEAD | /usr/share/clang/clang-format-diff.py -p1 -style=Google -i

