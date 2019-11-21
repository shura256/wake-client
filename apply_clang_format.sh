#!/bin/bash
find src -maxdepth 1 -type f \( -name "*.h" -o -name "*.cpp" \) -print -exec clang-format -i -style=Google {} \;

