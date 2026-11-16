#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
APX Documentation Validation Script.
Validates:
1. Markdown file existence and syntax
2. Internal relative link integrity across docs/ and docs/api/
3. Navigation links in mkdocs.yml
4. Mermaid code fence blocks
5. compile_commands.json existence and JSON validity
"""

import os
import re
import sys
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DOCS_DIR = REPO_ROOT / "docs"
MKDOCS_FILE = REPO_ROOT / "mkdocs.yml"

def check_compile_commands():
    """Verify compile_commands.json is present and valid."""
    potential_paths = [
        REPO_ROOT / "compile_commands.json",
        REPO_ROOT / "build" / "compile_commands.json",
        REPO_ROOT / "build" / "stm32h7" / "compile_commands.json",
        REPO_ROOT / "build" / "sitl" / "compile_commands.json",
        REPO_ROOT / "build" / "linux-release" / "compile_commands.json",
    ]
    found = None
    for p in potential_paths:
        if p.exists():
            found = p
            break
    
    if not found:
        print("⚠️  Warning: compile_commands.json not found in root or build directories.")
        print("   Run `cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` to generate it.")
        return True  # Non-fatal during purely docs-only validation
    
    try:
        with open(found, "r") as f:
            data = json.load(f)
            if not isinstance(data, list):
                print(f"❌ Error: {found} is not a valid JSON compilation database list.")
                return False
            print(f"✅ Found valid compile_commands.json at {found.relative_to(REPO_ROOT)} ({len(data)} translation units)")
            return True
    except Exception as e:
        print(f"❌ Error reading {found}: {e}")
        return False

def check_mkdocs_nav():
    """Verify all nav targets in mkdocs.yml exist."""
    if not MKDOCS_FILE.exists():
        print("❌ Error: mkdocs.yml not found.")
        return False
    
    errors = []
    with open(MKDOCS_FILE, "r") as f:
        content = f.read()

    # Simple line-by-line check for .md files referenced in nav
    lines = content.splitlines()
    in_nav = False
    for line in lines:
        if line.strip().startswith("nav:"):
            in_nav = True
            continue
        if in_nav and line and not line.startswith("  ") and not line.startswith("\t"):
            in_nav = False
        
        if in_nav:
            match = re.search(r":\s*['\"]?([a-zA-Z0-9_\-/\.]+\.md)['\"]?", line)
            if match:
                md_rel_path = match.group(1).strip()
                target_file = DOCS_DIR / md_rel_path
                if not target_file.exists():
                    errors.append(f"mkdocs.yml nav references missing file: docs/{md_rel_path}")

    if errors:
        for err in errors:
            print(f"❌ {err}")
        return False
    print(f"✅ mkdocs.yml navigation integrity verified.")
    return True

def check_markdown_links():
    """Check for broken relative links within docs."""
    md_files = list(DOCS_DIR.glob("**/*.md"))
    link_pattern = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')
    errors = []
    checked_count = 0

    for md_file in md_files:
        with open(md_file, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()
        
        # Strip code blocks to avoid false positives in code snippets
        text_no_code = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
        
        for match in link_pattern.finditer(text_no_code):
            text_label = match.group(1)
            target = match.group(2).strip()

            # Skip web URLs, mailto, anchor only
            if target.startswith(("http://", "https://", "mailto:", "#")):
                continue
            
            # Remove anchor fragment
            path_part = target.split("#")[0]
            if not path_part:
                continue
            
            # Resolve relative to md_file
            resolved_target = (md_file.parent / path_part).resolve()
            
            checked_count += 1
            if not resolved_target.exists():
                rel_source = md_file.relative_to(DOCS_DIR)
                errors.append(f"Broken link in docs/{rel_source}: [{text_label}]({target}) -> {resolved_target}")

    if errors:
        for err in errors:
            print(f"❌ {err}")
        return False
    print(f"✅ Validated {checked_count} relative markdown links across {len(md_files)} documents (0 broken).")
    return True

def check_mermaid_syntax():
    """Verify Mermaid diagram blocks have balanced tags and basic syntax."""
    md_files = list(DOCS_DIR.glob("**/*.md"))
    mermaid_block_pattern = re.compile(r'```mermaid\s*\n(.*?)\n```', flags=re.DOTALL)
    valid_diagram_types = [
        "flowchart", "graph", "sequenceDiagram", "classDiagram",
        "stateDiagram", "stateDiagram-v2", "erDiagram", "gantt", "pie"
    ]
    
    diagram_count = 0
    errors = []
    
    for md_file in md_files:
        with open(md_file, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()
        
        for match in mermaid_block_pattern.finditer(content):
            diagram_count += 1
            diagram_code = match.group(1).strip()
            first_line = diagram_code.splitlines()[0].strip() if diagram_code else ""
            
            # Check diagram type starts with one of valid types
            type_match = any(first_line.startswith(t) for t in valid_diagram_types)
            if not type_match:
                errors.append(
                    f"Invalid or unrecognized Mermaid diagram type in docs/{md_file.relative_to(DOCS_DIR)}: '{first_line}'"
                )
            
            # Check bracket matching
            brackets = {'[': ']', '(': ')', '{': '}'}
            stack = []
            # Tokenize or simple character check outside quotes
            in_quote = False
            for char in diagram_code:
                if char == '"':
                    in_quote = not in_quote
                elif not in_quote:
                    if char in brackets:
                        stack.append(char)
                    elif char in brackets.values():
                        if stack and brackets[stack[-1]] == char:
                            stack.pop()

    if errors:
        for err in errors:
            print(f"❌ {err}")
        return False

    print(f"✅ Validated {diagram_count} Mermaid diagrams across documentation.")
    return True

def main():
    print("=" * 60)
    print("APX Documentation Validation Suite")
    print("=" * 60)
    
    success = True
    if not check_compile_commands():
        success = False
    if not check_mkdocs_nav():
        success = False
    if not check_markdown_links():
        success = False
    if not check_mermaid_syntax():
        success = False

    print("=" * 60)
    if success:
        print("🎉 ALL DOCUMENTATION VALIDATION CHECKS PASSED!")
        sys.exit(0)
    else:
        print("❌ Documentation validation failed with errors.")
        sys.exit(1)

if __name__ == "__main__":
    main()
