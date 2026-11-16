#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
APX C++ API Documentation Generator.

Parses C++17 header files using robust AST tokenization and regex matching
to extract classes, structs, enums, public methods, inheritance, and docstrings.
Outputs GitHub/MkDocs-compliant Markdown pages into docs/api/.
"""

import os
import re
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Member:
    name: str
    kind: str  # method, field, enum_val
    return_type: str = ""
    args: str = ""
    comment: str = ""


@dataclass
class CppEntity:
    kind: str  # class, struct, enum
    name: str
    namespace: str
    header_file: str
    brief: str = ""
    base_classes: List[str] = field(default_factory=list)
    members: List[Member] = field(default_factory=list)


def clean_comment(comment_raw: str) -> str:
    """Clean C++ comments (///, /**, //) into concise markdown."""
    lines = []
    for line in comment_raw.splitlines():
        line = line.strip()
        if line.startswith("///"):
            line = line[3:].strip()
        elif line.startswith("//"):
            line = line[2:].strip()
        elif line.startswith("/**"):
            line = line[3:].strip()
        elif line.startswith("/*"):
            line = line[2:].strip()
        elif line.endswith("*/"):
            line = line[:-2].strip()
        elif line.startswith("*"):
            line = line[1:].strip()
        if line:
            lines.append(line)
    return " ".join(lines)


def extract_classes_and_structs(content: str, rel_path: str, current_ns: str) -> List[CppEntity]:
    """Extract classes and structs using balanced brace parsing."""
    entities: List[CppEntity] = []
    
    # Negative lookbehind (?<!enum\s) ensures we do not match `enum class Foo`
    decl_pattern = re.compile(
        r"((?:/\*\*.*?\*/|//[^\n]*\n|\s)*)(?<!enum\s)\b(class|struct)\s+([A-Za-z0-9_]+)(?:\s*:\s*([^{;]+))?\s*\{",
        re.DOTALL
    )
    
    pos = 0
    while True:
        m = decl_pattern.search(content, pos)
        if not m:
            break
        
        doc_comment = m.group(1) or ""
        kind = m.group(2)
        name = m.group(3)
        bases = m.group(4) or ""
        open_brace_idx = m.end() - 1
        
        # Balance braces to find true class end
        brace_count = 1
        i = open_brace_idx + 1
        in_line_comment = False
        in_block_comment = False
        in_string = False
        
        while i < len(content) and brace_count > 0:
            c = content[i]
            if in_line_comment:
                if c == '\n':
                    in_line_comment = False
            elif in_block_comment:
                if c == '*' and i + 1 < len(content) and content[i+1] == '/':
                    in_block_comment = False
                    i += 1
            elif in_string:
                if c == '\\':
                    i += 1
                elif c == '"':
                    in_string = False
            else:
                if c == '/' and i + 1 < len(content) and content[i+1] == '/':
                    in_line_comment = True
                    i += 1
                elif c == '/' and i + 1 < len(content) and content[i+1] == '*':
                    in_block_comment = True
                    i += 1
                elif c == '"':
                    in_string = True
                elif c == '{':
                    brace_count += 1
                elif c == '}':
                    brace_count -= 1
            i += 1
        
        if brace_count == 0:
            body = content[open_brace_idx + 1 : i - 1]
            pos = i
            
            # Skip forward declarations, internals or test helpers
            if name.startswith("_") or "detail" in name.lower():
                continue
            
            entity = CppEntity(kind, name, current_ns, rel_path)
            entity.brief = clean_comment(doc_comment)
            if bases:
                base_list = [b.strip().split()[-1] for b in bases.split(",") if b.strip()]
                entity.base_classes = base_list
            
            # Extract public section
            public_body = body
            if "public:" in body:
                parts = body.split("public:")
                public_sections = []
                for p in parts[1:]:
                    sec = p.split("private:")[0].split("protected:")[0]
                    public_sections.append(sec)
                public_body = "\n".join(public_sections)
            
            # Find public methods and constructors
            method_pattern = re.compile(
                r"(?:(/\*\*.*?\*/|//[^\n]*\n)*)\s*(?:virtual\s+|static\s+|explicit\s+|constexpr\s+|inline\s+)*"
                r"(?:(~?[A-Za-z0-9_]+)\s*\(([^)]*)\)|([A-Za-z0-9_:<>\*&]+)\s+([A-Za-z0-9_]+)\s*\(([^)]*)\))"
                r"(?:\s*const)?(?:\s*override)?(?:\s*noexcept)?(?:\s*=\s*0)?(?:\s*\{[^}]*\})?\s*;",
                re.DOTALL
            )
            for mm in method_pattern.finditer(public_body):
                m_doc = clean_comment(mm.group(1) or "")
                if mm.group(2):
                    ret_type = ""
                    fn_name = mm.group(2)
                    fn_args = mm.group(3).strip().replace("\n", " ")
                else:
                    ret_type = mm.group(4) or "void"
                    fn_name = mm.group(5)
                    fn_args = mm.group(6).strip().replace("\n", " ")
                fn_args = re.sub(r"\s+", " ", fn_args)
                if fn_name not in ("if", "for", "while", "switch", "return"):
                    entity.members.append(Member(fn_name, "method", ret_type, fn_args, m_doc))
            
            entities.append(entity)
        else:
            pos = m.end()
            
    return entities


def extract_enums(content: str, rel_path: str, current_ns: str) -> List[CppEntity]:
    """Extract enum and enum class definitions."""
    entities: List[CppEntity] = []
    enum_pattern = re.compile(
        r"((?:/\*\*.*?\*/|//[^\n]*\n|\s)*)enum\s+(?:class\s+)?([A-Za-z0-9_]+)(?:\s*:\s*[A-Za-z0-9_]+)?\s*\{([^}]*)\};",
        re.DOTALL
    )
    for m in enum_pattern.finditer(content):
        doc_comment = m.group(1) or ""
        name = m.group(2)
        body = m.group(3)
        entity = CppEntity("enum", name, current_ns, rel_path)
        entity.brief = clean_comment(doc_comment)
        for item in body.split(","):
            item = item.strip()
            if item:
                val_name = item.split("=")[0].strip()
                if val_name and not val_name.startswith("//"):
                    entity.members.append(Member(val_name, "enum_val"))
        entities.append(entity)
    return entities


def parse_header_file(header_path: str, repo_root: str) -> List[CppEntity]:
    """Parse header file and return all discovered entities."""
    try:
        with open(header_path, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()
    except Exception:
        return []

    rel_path = os.path.relpath(header_path, repo_root)

    # Determine namespaces
    ns_matches = re.findall(r"namespace\s+([a-zA-Z0-9_:]+)\s*\{", content)
    current_ns = "::".join(ns_matches) if ns_matches else "apx"

    entities: List[CppEntity] = []
    entities.extend(extract_classes_and_structs(content, rel_path, current_ns))
    entities.extend(extract_enums(content, rel_path, current_ns))
    return entities


def generate_module_api_doc(module_title: str, module_ns: str, entities: List[CppEntity]) -> str:
    """Generate Markdown documentation for a group of entities."""
    lines = [
        f"# {module_title} API Reference",
        "",
        f"Namespace: `{module_ns}`",
        "",
        "## Summary",
        "",
        f"This page documents all public classes, data structures, and enumerations for `{module_title}`.",
        "",
        "| Symbol | Kind | Declared In | Description |",
        "|---|---|---|---|",
    ]

    # Deduplicate entities with same name and kind
    unique_entities = {}
    for e in entities:
        key = (e.name, e.kind)
        if key not in unique_entities or len(e.members) > len(unique_entities[key].members):
            unique_entities[key] = e
    sorted_entities = sorted(unique_entities.values(), key=lambda x: x.name)

    for e in sorted_entities:
        anchor = e.name.lower()
        desc = e.brief if e.brief else f"{e.kind.capitalize()} definition."
        lines.append(f"| [`{e.name}`](#{anchor}) | `{e.kind}` | `{e.header_file}` | {desc[:80]} |")

    lines.append("")
    lines.append("---")
    lines.append("")

    for e in sorted_entities:
        lines.append(f"## {e.name}")
        lines.append(f"**Kind:** `{e.kind}`  ")
        lines.append(f"**Namespace:** `{e.namespace}`  ")
        lines.append(f"**Header:** `{e.header_file}`  ")

        if e.base_classes:
            lines.append(f"**Inherits from:** " + ", ".join([f"`{b}`" for b in e.base_classes]) + "  ")

        if e.brief:
            lines.append("")
            lines.append(f"> {e.brief}")

        lines.append("")

        if e.kind == "enum":
            lines.append("### Enumerators")
            lines.append("| Value | Description |")
            lines.append("|---|---|")
            for m in e.members:
                lines.append(f"| `{m.name}` | Enumerator value |")
            lines.append("")
        else:
            if e.members:
                lines.append("### Public Methods & Members")
                lines.append("| Return Type | Signature | Description |")
                lines.append("|---|---|---|")
                for m in e.members:
                    sig = f"`{m.name}({m.args})`"
                    doc = m.comment if m.comment else "Member function."
                    lines.append(f"| `{m.return_type}` | {sig} | {doc} |")
                lines.append("")

        lines.append("---")
        lines.append("")

    return "\n".join(lines)


def main():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    api_docs_dir = os.path.join(repo_root, "docs", "api")
    os.makedirs(api_docs_dir, exist_ok=True)

    header_dirs = [
        ("Flight Control", "apx::control", os.path.join(repo_root, "modules", "flight_control"), "control.md"),
        ("State Estimation", "apx::estimator", os.path.join(repo_root, "modules", "estimator"), "estimator.md"),
        ("Flight Modes", "apx::modes", os.path.join(repo_root, "modules", "modes"), "modes.md"),
        ("GCS & MAVLink", "apx::gcs", os.path.join(repo_root, "modules", "gcs"), "gcs.md"),
        ("Hardware Abstraction Layer", "apx::hal", os.path.join(repo_root, "hal", "include"), "hal.md"),
        ("Kernel Subsystem", "apx::kernel", os.path.join(repo_root, "kernel", "include"), "kernel.md"),
        ("Device Drivers", "apx::drivers", os.path.join(repo_root, "drivers"), "drivers.md"),
        ("Platform Support", "apx::platform", os.path.join(repo_root, "platforms", "stm32h7"), "platform.md"),
    ]

    all_entities_count = 0
    index_entries = []

    for title, ns, path, doc_filename in header_dirs:
        entities: List[CppEntity] = []
        if os.path.exists(path):
            for root, _, files in os.walk(path):
                # Exclude third-party / mavlink generated / toolchain
                if "mavlink/common" in root or "mavlink/minimal" in root or "mavlink/standard" in root or "toolchain" in root:
                    continue
                for f in files:
                    if f.endswith((".hpp", ".h")):
                        header_full = os.path.join(root, f)
                        entities.extend(parse_header_file(header_full, repo_root))

        out_path = os.path.join(api_docs_dir, doc_filename)
        with open(out_path, "w", encoding="utf-8") as out_f:
            out_f.write(generate_module_api_doc(title, ns, entities))

        all_entities_count += len(entities)
        index_entries.append((title, ns, doc_filename, len(entities)))
        print(f"[APX DocGen] Extracted {len(entities)} C++ entities for {title} -> {doc_filename}")

    # Generate docs/api/index.md
    index_lines = [
        "# APX C++ API Documentation Index",
        "",
        "Welcome to the formal C++ API reference for the APX Flight Controller codebase.",
        "This API reference is extracted directly from the C++17 source and header files,",
        "indexed by architectural layer and namespace.",
        "",
        "## Subsystem Reference Modules",
        "",
        "| Layer / Subsystem | Primary Namespace | API Reference Link | Exported Symbols |",
        "|---|---|---|:---:|",
    ]

    for title, ns, filename, count in index_entries:
        index_lines.append(f"| **{title}** | `{ns}` | [{title}]({filename}) | {count} |")

    index_lines.extend([
        "",
        f"**Total Documented C++ Symbols:** `{all_entities_count}`",
        "",
        "## Standards & Conventions",
        "- **C++ Standard:** C++17 (`std::array`, `std::atomic`, `constexpr`, `std::string_view`)",
        "- **Memory Policy:** Zero heap allocation in flight loops; static sizing or stack-allocated frames",
        "- **Concurrency:** Lock-free pub/sub bus (`apx::kernel::Topic`) and mutex-guarded GCS streams",
        "- **Platform Portability:** Pure virtual interfaces in `apx::hal` implemented by POSIX, SITL, and bare-metal STM32H7",
    ])

    with open(os.path.join(api_docs_dir, "index.md"), "w", encoding="utf-8") as out_f:
        out_f.write("\n".join(index_lines) + "\n")

    print(f"[APX DocGen] Complete API reference generated: {all_entities_count} total symbols.")


if __name__ == "__main__":
    main()
