#!/bin/sh
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
# Build and run the IO coprocessor driver host test.
set -e
cd "$(dirname "$0")"
OUT="${TMPDIR:-/tmp}/apx_iomcu_test_$$"
${CXX:-c++} -std=c++17 -Wall -Wextra -Werror -I../../hal/include -I../../drivers/iomcu \
    test_io_coprocessor.cpp ../../drivers/iomcu/io_coprocessor.cpp -o "$OUT"
"$OUT"; rc=$?
rm -f "$OUT"
exit $rc
