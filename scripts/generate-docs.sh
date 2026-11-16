#!/usr/bin/env bash
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=============================================================================="
echo "APX Documentation Generator"
echo "Repository Root: ${REPO_ROOT}"
echo "=============================================================================="

cd "${REPO_ROOT}"

# 1. Ensure compilation database exists for Clang / AST analysis
if [ ! -f "${REPO_ROOT}/compile_commands.json" ]; then
    if [ -f "${REPO_ROOT}/build/stm32h7/compile_commands.json" ]; then
        ln -sf "${REPO_ROOT}/build/stm32h7/compile_commands.json" "${REPO_ROOT}/compile_commands.json"
        echo "🔗 Linked compile_commands.json from build/stm32h7/"
    elif [ -f "${REPO_ROOT}/build/compile_commands.json" ]; then
        ln -sf "${REPO_ROOT}/build/compile_commands.json" "${REPO_ROOT}/compile_commands.json"
        echo "🔗 Linked compile_commands.json from build/"
    else
        echo "⚙️  Generating compilation database via CMake..."
        cmake -B build/doc_gen -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DAPX_BUILD_SITL=ON -DAPX_BUILD_TESTS=ON > /dev/null 2>&1 || true
        if [ -f "build/doc_gen/compile_commands.json" ]; then
            ln -sf "${REPO_ROOT}/build/doc_gen/compile_commands.json" "${REPO_ROOT}/compile_commands.json"
            echo "🔗 Generated and linked compile_commands.json"
        fi
    fi
fi

# 2. Extract C++ API documentation
echo ""
echo "📚 Extracting C++ API documentation from codebase..."
python3 "${REPO_ROOT}/scripts/extract_cpp_api.py"

# 3. Validate Markdown links, Mermaid diagrams, and navigation
echo ""
echo "🔍 Validating documentation integrity..."
python3 "${REPO_ROOT}/scripts/validate-docs.py"

# 4. Build static HTML documentation
echo ""
echo "🏗️  Building documentation site with MkDocs..."
MKDOCS_BIN="mkdocs"
if ! command -v mkdocs &> /dev/null; then
    if [ -f "${HOME}/.local/bin/mkdocs" ]; then
        MKDOCS_BIN="${HOME}/.local/bin/mkdocs"
    else
        echo "❌ mkdocs executable not found in PATH or ~/.local/bin."
        echo "   Please install via: pip install mkdocs mkdocs-material"
        exit 1
    fi
fi

"${MKDOCS_BIN}" build --strict

echo ""
echo "=============================================================================="
echo "🎉 Documentation build successfully completed!"
echo "   Output directory: ${REPO_ROOT}/site"
echo "   Local preview:    ${MKDOCS_BIN} serve"
echo "=============================================================================="
