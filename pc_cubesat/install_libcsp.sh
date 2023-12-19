#!/bin/bash
cd libcsp
cmake -GNinja -DCSP_ENABLE_PYTHON3_BINDINGS=ON
ninja
mv libcsp_py3.*.so ../libcsp_py3.so
