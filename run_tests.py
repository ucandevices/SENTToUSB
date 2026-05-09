#!/usr/bin/env python
"""
Quick test runner for SENT integration tests.
Provides convenient shortcuts for common test scenarios.
"""

import subprocess
import sys
from pathlib import Path

def run_command(cmd_list, description):
    """Run a command and report results."""
    print(f"\n{'='*70}")
    print(f"  {description}")
    print(f"{'='*70}\n")
    
    try:
        result = subprocess.run(cmd_list, check=True)
        return result.returncode == 0
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Command failed with exit code {e.returncode}")
        return False
    except FileNotFoundError:
        print(f"\n❌ Command not found. Make sure pytest is installed:")
        print("   pip install -r test_requirements.txt")
        return False

def main():
    """Main test runner."""
    if len(sys.argv) < 2:
        command = "all"
    else:
        command = sys.argv[1].lower()
    
    test_file = Path("test_sent_integration.py")
    if not test_file.exists():
        print(f"❌ Test file not found: {test_file}")
        sys.exit(1)
    
    # Base pytest command
    base_cmd = [sys.executable, "-m", "pytest", str(test_file), "-v", "-s"]
    
    success = False
    
    if command == "all":
        success = run_command(
            base_cmd + ["--tb=short"],
            "Running all integration tests"
        )
    
    elif command == "quick":
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_devices_present_and_responsive",
                "::TestSENTIntegration::test_can_enter_rx_mode",
                "::TestSENTIntegration::test_can_enter_tx_mode",
                "--tb=short"
            ],
            "Running quick connectivity tests"
        )
    
    elif command in ["3", "3.0", "tick3"]:
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_tx_rx_with_tick_time[3.0]",
                "--tb=short"
            ],
            "Running test with 3.0µs tick time"
        )
    
    elif command in ["6", "6.0", "tick6"]:
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_tx_rx_with_tick_time[6.0]",
                "--tb=short"
            ],
            "Running test with 6.0µs tick time"
        )
    
    elif command in ["9", "9.0", "tick9"]:
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_tx_rx_with_tick_time[9.0]",
                "--tb=short"
            ],
            "Running test with 9.0µs tick time"
        )
    
    elif command in ["12", "12.0", "tick12"]:
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_tx_rx_with_tick_time[12.0]",
                "--tb=short"
            ],
            "Running test with 12.0µs tick time"
        )
    
    elif command == "sequence":
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_multiple_frames_sequence",
                "--tb=short"
            ],
            "Running frame sequence test"
        )
    
    elif command in ["framesize", "dlc", "nibble"]:
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_all_frame_sizes_with_default_tick",
                "--tb=short"
            ],
            "Running frame size / nibble count test"
        )
    
    elif command == "combined":
        success = run_command(
            base_cmd + [
                "::TestSENTIntegration::test_tx_rx_with_tick_and_frame_size",
                "--tb=short"
            ],
            "Running combined tick+frame size tests (16 parametrizations)"
        )
    
    elif command == "debug":
        success = run_command(
            base_cmd + ["--log-cli-level=DEBUG", "--tb=long"],
            "Running with debug logging"
        )
    
    elif command == "report":
        success = run_command(
            [sys.executable, "-m", "pytest", str(test_file), "-v", 
             "--tb=short", "--html=test_report.html", "--self-contained-html"],
            "Generating HTML test report"
        )
        if success:
            print("\n✅ Report generated: test_report.html")
    
    elif command in ["help", "-h", "--help", "?"]:
        print("""
SENT Integration Test Runner

Usage: python run_tests.py [command]

Commands:
  all              Run all tests (default)
  quick            Run basic connectivity tests only
  3 / 3.0 / tick3  Run tests with 3.0µs tick time
  6 / 6.0 / tick6  Run tests with 6.0µs tick time
  9 / 9.0 / tick9  Run tests with 9.0µs tick time
  12 / 12.0 / tick12  Run tests with 12.0µs tick time
  sequence         Run frame sequence test
  framesize/dlc/nibble  Run frame size/DLC/nibble count test
  combined         Run combined tick+frame size tests (16 variants)
  debug            Run with debug logging
  report           Generate HTML test report
  help             Show this help message

Examples:
  python run_tests.py                  # Run all tests
  python run_tests.py quick            # Quick connectivity check
  python run_tests.py 3                # Test 3µs tick time only
  python run_tests.py framesize        # Test different frame sizes
  python run_tests.py combined         # Test all tick × frame combinations
  python run_tests.py debug            # Run with detailed logging
  python run_tests.py report           # Generate HTML report
        """)
        sys.exit(0)
    
    else:
        print(f"❌ Unknown command: {command}")
        print("   Run 'python run_tests.py help' for available commands")
        sys.exit(1)
    
    # Exit with appropriate code
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
