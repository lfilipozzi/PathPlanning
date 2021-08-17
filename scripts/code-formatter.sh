#!/bin/bash

find -name "*.cpp" -not -path "*/external/*" -not -path "./build/*" -exec clang-format -style=file -i {} \;
find -name "*.h"   -not -path "*/external/*" -not -path "./build/*" -exec clang-format -style=file -i {} \;
