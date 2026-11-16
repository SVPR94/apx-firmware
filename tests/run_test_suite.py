#!/usr/bin/env python3
# SPDX-License-Identifier: CC-BY-NC-4.0
# Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
# Author: Savita Pawar (SVPR94)
# Created: 2026
"""
APX Unified Test Suite Runner.
Executes both C++ GoogleTest unit tests and SITL flight scenario integration tests.
Generates structured Markdown reports (for GitHub Step Summary) and JUnit XML reports.
"""

import argparse
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class TestResult:
    name: str
    category: str  # 'Unit' or 'SITL Integration'
    passed: bool
    duration_s: float
    message: str = ""
    stdout: str = ""
    stderr: str = ""
    metrics: Dict[str, str] = field(default_factory=dict)


class APXTestRunner:
    def __init__(self, root_dir: Path, build_dir: Optional[Path] = None, verbose: bool = False):
        self.root_dir = root_dir
        self.verbose = verbose
        self.results: List[TestResult] = []

        # Autodetect build directory if not specified
        if build_dir:
            self.build_dir = build_dir
        else:
            candidates = [
                self.root_dir / "build" / "sitl",
                self.root_dir / "build" / "tests",
                self.root_dir / "build",
            ]
            self.build_dir = next((d for d in candidates if (d / "tests" / "unit" / "run_unit_tests").exists() or (d / "run_unit_tests").exists()), candidates[0])

    def log(self, msg: str, color: Optional[str] = None):
        colors = {
            "green": "\033[92m",
            "red": "\033[91m",
            "yellow": "\033[93m",
            "blue": "\033[94m",
            "bold": "\033[1m",
            "reset": "\033[0m",
        }
        if color and sys.stdout.isatty():
            print(f"{colors.get(color, '')}{msg}{colors['reset']}")
        else:
            print(msg)

    def run_unit_tests(self) -> bool:
        self.log("\n==================================================", "bold")
        self.log(" [1/2] RUNNING C++ GOOGLETEST UNIT TESTS", "blue")
        self.log("==================================================", "bold")

        # Locate unit test binary
        possible_bins = [
            self.build_dir / "tests" / "unit" / "run_unit_tests",
            self.build_dir / "run_unit_tests",
            self.root_dir / "build" / "tests" / "tests" / "unit" / "run_unit_tests",
            self.root_dir / "build" / "sitl" / "tests" / "unit" / "run_unit_tests",
        ]
        test_bin = next((b for b in possible_bins if b.exists()), None)

        if not test_bin:
            self.log(f"⚠️  Unit test binary not found in {self.build_dir}. Attempting CTest...", "yellow")
            return self._run_ctest()

        self.log(f"Executing: {test_bin}")
        t0 = time.time()
        try:
            proc = subprocess.run(
                [str(test_bin)],
                cwd=self.root_dir,
                capture_output=True,
                text=True,
                timeout=60,
            )
            duration = time.time() - t0
            passed = (proc.returncode == 0)

            # Parse GoogleTest individual test cases
            test_cases = re.findall(r"\[ RUN      \] ([\w\.]+)\n\[\s*(OK|FAILED)\s*\] \1 \((\d+) ms\)", proc.stdout)
            if test_cases:
                for tc_name, status, tc_ms in test_cases:
                    tc_passed = (status == "OK")
                    self.results.append(
                        TestResult(
                            name=f"Unit::{tc_name}",
                            category="Unit",
                            passed=tc_passed,
                            duration_s=float(tc_ms) / 1000.0,
                            message=status,
                            stdout=proc.stdout if not tc_passed else "",
                        )
                    )
            else:
                self.results.append(
                    TestResult(
                        name="Unit::GoogleTest_Suite",
                        category="Unit",
                        passed=passed,
                        duration_s=duration,
                        message="Passed" if passed else f"Exit code {proc.returncode}",
                        stdout=proc.stdout,
                        stderr=proc.stderr,
                    )
                )

            if passed:
                self.log(f"✅ All unit tests PASSED ({duration:.3f}s)", "green")
            else:
                self.log(f"❌ Unit tests FAILED with code {proc.returncode}", "red")
                if self.verbose:
                    print(proc.stdout)
                    print(proc.stderr)
            return passed

        except Exception as e:
            duration = time.time() - t0
            self.results.append(
                TestResult(
                    name="Unit::GoogleTest_Suite",
                    category="Unit",
                    passed=False,
                    duration_s=duration,
                    message=f"Execution error: {e}",
                )
            )
            self.log(f"❌ Error running unit tests: {e}", "red")
            return False

    def _run_ctest(self) -> bool:
        t0 = time.time()
        try:
            proc = subprocess.run(
                ["ctest", "--test-dir", str(self.build_dir), "--output-on-failure"],
                cwd=self.root_dir,
                capture_output=True,
                text=True,
                timeout=60,
            )
            duration = time.time() - t0
            passed = (proc.returncode == 0)
            self.results.append(
                TestResult(
                    name="Unit::CTest",
                    category="Unit",
                    passed=passed,
                    duration_s=duration,
                    message="Passed" if passed else "CTest failed",
                    stdout=proc.stdout,
                    stderr=proc.stderr,
                )
            )
            return passed
        except Exception as e:
            self.log(f"❌ CTest failed to execute: {e}", "red")
            return False

    @staticmethod
    def _cleanup_processes():
        for proc in ["apx_sitl", "apx_sim_physics_runner", "gz sim", "gz-transport"]:
            subprocess.run(["pkill", "-9", "-f", proc], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        time.sleep(0.5)

    def run_sitl_tests(self, filter_name: Optional[str] = None) -> bool:
        self.log("\n==================================================", "bold")
        self.log(" [2/2] RUNNING SITL FLIGHT INTEGRATION TESTS", "blue")
        self.log("==================================================", "bold")

        sitl_dir = self.root_dir / "sitl"
        sitl_scripts = [
            ("sitl/test_mission_item_speed.py", "Per-Mission-Item Speed & Dynamic DO_CHANGE_SPEED"),
            ("sitl/test_waypoint_approach_and_braking.py", "Trajectory Tracking, Waypoint Braking & Zero-Drop Tilt Compensation"),
            ("sitl/test_gcs_mission_and_joystick.py", "GCS Mission Upload/Execution & Loiter Virtual Joystick Override"),
            ("sitl/test_auto_path_tracking.py", "Autonomous S-Curve Path & Waypoint Verification"),
            ("sitl/test_loiter_landing.py", "Position Hold Loiter & Autonomous Touchdown Disarm"),
            ("sitl/test_sprayer.py", "Agricultural Sprayer Payload & Flow Rate Control"),
        ]

        all_passed = True
        self._cleanup_processes()
        for rel_path, desc in sitl_scripts:
            script_path = self.root_dir / rel_path
            if not script_path.exists():
                continue

            test_key = script_path.stem
            if filter_name and filter_name not in test_key and filter_name not in rel_path:
                continue

            self._cleanup_processes()
            self.log(f"\n▶ Running {test_key}: {desc}...", "bold")
            t0 = time.time()

            try:
                proc = subprocess.run(
                    [sys.executable, str(script_path)],
                    cwd=self.root_dir,
                    capture_output=True,
                    text=True,
                    timeout=180,
                )
                duration = time.time() - t0
                passed = (proc.returncode == 0)

                # Extract key metrics from test output
                metrics = {}
                for line in proc.stdout.splitlines():
                    if "Max Recorded Speed" in line or "Speed =" in line:
                        metrics["Recorded Speed"] = line.strip()
                    elif "tracking error" in line.lower() or "avg tracking error" in line.lower():
                        metrics["Path Error"] = line.strip()
                    elif "altitude drop" in line.lower() or "max drop" in line.lower():
                        metrics["Altitude Drop"] = line.strip()
                    elif "touchdown confirmed" in line.lower():
                        metrics["Landing"] = "Confirmed"

                if passed:
                    self.log(f"  ✅ {test_key} PASSED ({duration:.1f}s)", "green")
                else:
                    self.log(f"  ❌ {test_key} FAILED (exit code {proc.returncode})", "red")
                    all_passed = False
                    if self.verbose:
                        print(proc.stdout[-1500:])
                        print(proc.stderr[-1000:])

                self.results.append(
                    TestResult(
                        name=f"SITL::{test_key}",
                        category="SITL Integration",
                        passed=passed,
                        duration_s=duration,
                        message=desc,
                        stdout=proc.stdout,
                        stderr=proc.stderr,
                        metrics=metrics,
                    )
                )

            except subprocess.TimeoutExpired:
                duration = time.time() - t0
                self.log(f"  ❌ {test_key} TIMEOUT after {duration:.1f}s", "red")
                all_passed = False
                self.results.append(
                    TestResult(
                        name=f"SITL::{test_key}",
                        category="SITL Integration",
                        passed=False,
                        duration_s=duration,
                        message=f"{desc} (Timeout)",
                    )
                )
            except Exception as e:
                duration = time.time() - t0
                self.log(f"  ❌ {test_key} EXCEPTION: {e}", "red")
                all_passed = False
                self.results.append(
                    TestResult(
                        name=f"SITL::{test_key}",
                        category="SITL Integration",
                        passed=False,
                        duration_s=duration,
                        message=f"Execution error: {e}",
                    )
                )

        self._cleanup_processes()
        return all_passed

    def print_summary(self):
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        total_time = sum(r.duration_s for r in self.results)

        self.log("\n" + "=" * 70, "bold")
        self.log("                    APX TEST EXECUTION SUMMARY", "bold")
        self.log("=" * 70, "bold")
        self.log(f" Total Tests Run: {total:3d}  |  Passed: {passed:3d}  |  Failed: {failed:3d}  |  Time: {total_time:.2f}s")
        self.log("-" * 70)

        for r in self.results:
            status_str = "\033[92m[PASS]\033[0m" if r.passed and sys.stdout.isatty() else ("[PASS]" if r.passed else "[FAIL]")
            metric_str = f" ({', '.join(f'{k}: {v}' for k, v in list(r.metrics.items())[:2])})" if r.metrics else ""
            print(f"{status_str:10s} {r.name:<45s} {r.duration_s:6.2f}s{metric_str}")

        self.log("=" * 70, "bold")
        if failed == 0:
            self.log("🎉 ALL APX TESTS PASSED SUCCESSFULLY! Ready for release.", "green")
        else:
            self.log(f"❌ {failed} TEST(S) FAILED. Please inspect logs above.", "red")

    def generate_markdown_report(self, report_path: Path):
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        total_time = sum(r.duration_s for r in self.results)
        status_banner = "✅ **ALL TESTS PASSED**" if failed == 0 else f"❌ **{failed} TESTS FAILED**"

        lines = [
            "# 🛸 APX Firmware Test Suite Execution Report",
            "",
            f"**Overall Status:** {status_banner}  ",
            f"**Execution Timestamp:** `{time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime())}`  ",
            f"**Total Tests Run:** `{total}` | **Passed:** `{passed}` | **Failed:** `{failed}` | **Total Duration:** `{total_time:.2f}s`",
            "",
            "## 📊 Summary by Category",
            "",
            "| Category | Total | Passed | Failed | Success Rate |",
            "| :--- | :---: | :---: | :---: | :---: |",
        ]

        categories = sorted(list(set(r.category for r in self.results)))
        for cat in categories:
            cat_tests = [r for r in self.results if r.category == cat]
            c_total = len(cat_tests)
            c_pass = sum(1 for r in cat_tests if r.passed)
            c_fail = c_total - c_pass
            rate = (c_pass / c_total * 100.0) if c_total > 0 else 0.0
            lines.append(f"| **{cat}** | {c_total} | {c_pass} | {c_fail} | {rate:.1f}% |")

        lines.extend([
            "",
            "## 📋 Detailed Test Results",
            "",
            "| Test Name | Category | Status | Duration | Description / Key Metrics |",
            "| :--- | :--- | :---: | :---: | :--- |",
        ])

        for r in self.results:
            icon = "✅ Pass" if r.passed else "❌ Fail"
            metrics_snippet = "; ".join(f"{k}: `{v}`" for k, v in r.metrics.items()) if r.metrics else r.message
            lines.append(f"| `{r.name}` | {r.category} | {icon} | {r.duration_s:.2f}s | {metrics_snippet} |")

        # Add collapsible failure logs if any test failed
        failures = [r for r in self.results if not r.passed]
        if failures:
            lines.extend([
                "",
                "## ⚠️ Failure Diagnostics & Logs",
                "",
            ])
            for f in failures:
                lines.extend([
                    f"<details><summary><b>❌ {f.name} (Click to expand)</b></summary>",
                    "",
                    "```text",
                    f.stdout[-2000:] if f.stdout else "(No stdout captured)",
                    f.stderr[-2000:] if f.stderr else "(No stderr captured)",
                    "```",
                    "</details>",
                    "",
                ])

        lines.extend([
            "",
            "---",
            "*Report generated automatically by APX Test Suite Runner (`tests/run_test_suite.py`).*",
        ])

        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text("\n".join(lines), encoding="utf-8")
        self.log(f"📝 Markdown report saved to: {report_path}", "green")

    def generate_junit_xml(self, xml_path: Path):
        testsuites = ET.Element("testsuites", name="APX_TestSuite")
        categories = sorted(list(set(r.category for r in self.results)))

        for cat in categories:
            cat_results = [r for r in self.results if r.category == cat]
            suite = ET.SubElement(
                testsuites,
                "testsuite",
                name=cat,
                tests=str(len(cat_results)),
                failures=str(sum(1 for r in cat_results if not r.passed)),
                time=f"{sum(r.duration_s for r in cat_results):.3f}",
            )
            for r in cat_results:
                tc = ET.SubElement(
                    suite,
                    "testcase",
                    name=r.name,
                    classname=r.category,
                    time=f"{r.duration_s:.3f}",
                )
                if not r.passed:
                    fail = ET.SubElement(tc, "failure", message=r.message)
                    fail.text = (r.stdout + "\n" + r.stderr).strip()

        xml_path.parent.mkdir(parents=True, exist_ok=True)
        tree = ET.ElementTree(testsuites)
        tree.write(xml_path, encoding="utf-8", xml_declaration=True)
        self.log(f"📝 JUnit XML report saved to: {xml_path}", "green")


def main():
    parser = argparse.ArgumentParser(description="APX Unified Test Suite Runner")
    parser.add_argument("--unit", action="store_true", help="Run C++ unit tests only")
    parser.add_argument("--sitl", action="store_true", help="Run SITL integration tests only")
    parser.add_argument("--all", action="store_true", help="Run both unit and SITL tests (default)")
    parser.add_argument("--sitl-test", type=str, default=None, help="Run specific SITL test script by name")
    parser.add_argument("--build-dir", type=Path, default=None, help="CMake build directory")
    parser.add_argument("--report-md", type=Path, default=None, help="Output path for Markdown summary report")
    parser.add_argument("--report-junit", type=Path, default=None, help="Output path for JUnit XML report")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose test execution output")

    args = parser.parse_args()
    root_dir = Path(__file__).resolve().parent.parent

    # Default to running all tests if neither --unit nor --sitl is specified
    run_unit = args.unit or args.all or (not args.unit and not args.sitl)
    run_sitl = args.sitl or args.all or (not args.unit and not args.sitl)

    runner = APXTestRunner(root_dir=root_dir, build_dir=args.build_dir, verbose=args.verbose)

    overall_ok = True
    if run_unit:
        ok = runner.run_unit_tests()
        overall_ok = overall_ok and ok

    if run_sitl:
        ok = runner.run_sitl_tests(filter_name=args.sitl_test)
        overall_ok = overall_ok and ok

    runner.print_summary()

    if args.report_md:
        runner.generate_markdown_report(args.report_md)

    if args.report_junit:
        runner.generate_junit_xml(args.report_junit)

    sys.exit(0 if overall_ok else 1)


if __name__ == "__main__":
    main()
