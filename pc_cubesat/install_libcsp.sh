#!/bin/bash
cd libcsp
cmake -GNinja -DCSP_ENABLE_PYTHON3_BINDINGS=ON
ninja
mv libcsp_py3.cpython-310-x86_64-linux-gnu.so ../libcsp_py3.so