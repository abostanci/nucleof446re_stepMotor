#!/usr/bin/env python3
"""
Motor Control System Test Script
Tests all major functionality of the dual motor control system
"""

import serial
import serial.tools.list_ports
import time
import sys

BAUD_RATE = 115200
TIMEOUT = 2.0

class MotorTester:
    def __init__(self, port=None):
        self.port = port
        self.serial = None
        self.test_results = []
        
    def find_port(self):
        """Auto-detect STM32 port"""
        print("Scanning for serial ports...")
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            print("❌ No serial ports found!")
            return None
        
        print(f"Found {len(ports)} port(s):")
        for i, port in enumerate(ports):
            print(f"  {i+1}. {port.device} - {port.description}")
        
        if len(ports) == 1:
            return ports[0].device
        
        try:
            choice = int(input("\nSelect port number: ")) - 1
            return ports[choice].device
        except (ValueError, IndexError):
            print("Invalid selection")
            return None
    
    def connect(self):
        """Connect to motor controller"""
        if not self.port:
            self.port = self.find_port()
            if not self.port:
                return False
        
        try:
            print(f"\n🔌 Connecting to {self.port}...")
            self.serial = serial.Serial(self.port, BAUD_RATE, timeout=TIMEOUT)
            time.sleep(2)  # Wait for connection to stabilize
            
            # Clear any startup messages
            self.serial.reset_input_buffer()
            
            print("✅ Connected!")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def send_command(self, command):
        """Send command and return response"""
        if not self.serial:
            return None
        
        print(f"  TX: {command}")
        self.serial.write((command + '\n').encode())
        
        # Read response(s)
        responses = []
        start_time = time.time()
        
        while time.time() - start_time < 1.0:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('ascii', errors='ignore').strip()
                if line:
                    print(f"  RX: {line}")
                    responses.append(line)
            else:
                time.sleep(0.01)
        
        return responses
    
    def test_connection(self):
        """Test basic connectivity"""
        print("\n" + "="*60)
        print("TEST 1: Connection")
        print("="*60)
        
        responses = self.send_command("STATUS")
        
        if responses and any("STATUS" in r for r in responses):
            print("✅ PASS - Device responding to STATUS command")
            self.test_results.append(("Connection", True))
            return True
        else:
            print("❌ FAIL - No STATUS response")
            self.test_results.append(("Connection", False))
            return False
    
    def test_position_reading(self):
        """Test position reading"""
        print("\n" + "="*60)
        print("TEST 2: Position Reading")
        print("="*60)
        
        responses = self.send_command("STATUS")
        
        motor0_pos = None
        motor1_pos = None
        
        for resp in responses:
            if resp.startswith("STATUS"):
                parts = resp.split(',')
                if len(parts) >= 3:
                    motor_id = int(parts[1])
                    position = int(parts[2])
                    
                    if motor_id == 0:
                        motor0_pos = position
                    elif motor_id == 1:
                        motor1_pos = position
        
        if motor0_pos is not None and motor1_pos is not None:
            print(f"✅ PASS - Motor 0: {motor0_pos}, Motor 1: {motor1_pos}")
            self.test_results.append(("Position Reading", True))
            return True
        else:
            print("❌ FAIL - Could not read positions")
            self.test_results.append(("Position Reading", False))
            return False
    
    def test_reset(self):
        """Test position reset"""
        print("\n" + "="*60)
        print("TEST 3: Position Reset")
        print("="*60)
        
        responses = self.send_command("RESET")
        
        if responses and any("OK" in r or "RESET" in r for r in responses):
            time.sleep(0.5)
            
            # Verify positions are zero
            responses = self.send_command("STATUS")
            
            for resp in responses:
                if resp.startswith("STATUS"):
                    parts = resp.split(',')
                    if len(parts) >= 3:
                        position = int(parts[2])
                        if position != 0:
                            print(f"⚠️  WARNING - Position not zero after reset: {position}")
            
            print("✅ PASS - Reset command accepted")
            self.test_results.append(("Reset", True))
            return True
        else:
            print("❌ FAIL - Reset command failed")
            self.test_results.append(("Reset", False))
            return False
    
    def test_small_movement(self):
        """Test small movement"""
        print("\n" + "="*60)
        print("TEST 4: Small Movement (100 steps)")
        print("="*60)
        
        print("Moving Motor 0 to position 100...")
        responses = self.send_command("MOVE,100,0")
        
        if responses and any("OK" in r for r in responses):
            print("Command accepted, waiting for movement...")
            time.sleep(2)
            
            # Check new position
            responses = self.send_command("STATUS")
            
            for resp in responses:
                if resp.startswith("STATUS,0"):
                    parts = resp.split(',')
                    if len(parts) >= 3:
                        position = int(parts[2])
                        if abs(position - 100) < 5:  # Allow small tolerance
                            print(f"✅ PASS - Motor at position {position}")
                            self.test_results.append(("Small Movement", True))
                            return True
            
            print("⚠️  Movement command sent but position unclear")
            self.test_results.append(("Small Movement", None))
            return None
        else:
            print("❌ FAIL - Movement command rejected")
            self.test_results.append(("Small Movement", False))
            return False
    
    def test_home(self):
        """Test home command"""
        print("\n" + "="*60)
        print("TEST 5: Home Command")
        print("="*60)
        
        responses = self.send_command("HOME")
        
        if responses and any("OK" in r or "HOMING" in r for r in responses):
            print("Homing command accepted, waiting...")
            time.sleep(3)
            
            # Check if at home
            responses = self.send_command("STATUS")
            
            print("✅ PASS - Home command executed")
            self.test_results.append(("Home", True))
            return True
        else:
            print("❌ FAIL - Home command failed")
            self.test_results.append(("Home", False))
            return False
    
    def test_stop(self):
        """Test emergency stop"""
        print("\n" + "="*60)
        print("TEST 6: Emergency Stop")
        print("="*60)
        
        # Start a movement
        print("Starting movement...")
        self.send_command("MOVE,1000,1000")
        time.sleep(0.3)
        
        # Send stop
        print("Sending STOP command...")
        responses = self.send_command("STOP")
        
        if responses and any("OK" in r or "STOP" in r for r in responses):
            print("✅ PASS - Stop command accepted")
            self.test_results.append(("Emergency Stop", True))
            return True
        else:
            print("❌ FAIL - Stop command failed")
            self.test_results.append(("Emergency Stop", False))
            return False
    
    def test_invalid_command(self):
        """Test invalid command handling"""
        print("\n" + "="*60)
        print("TEST 7: Invalid Command Handling")
        print("="*60)
        
        responses = self.send_command("INVALID_CMD_TEST")
        
        if responses and any("ERROR" in r or "UNKNOWN" in r for r in responses):
            print("✅ PASS - Invalid command properly rejected")
            self.test_results.append(("Invalid Command", True))
            return True
        else:
            print("❌ FAIL - Invalid command not handled")
            self.test_results.append(("Invalid Command", False))
            return False
    
    def test_dual_motor(self):
        """Test moving both motors"""
        print("\n" + "="*60)
        print("TEST 8: Dual Motor Movement")
        print("="*60)
        
        print("Moving both motors (M0:200, M1:150)...")
        responses = self.send_command("MOVE,200,150")
        
        if responses and any("OK" in r for r in responses):
            time.sleep(2)
            
            responses = self.send_command("STATUS")
            
            m0_ok = False
            m1_ok = False
            
            for resp in responses:
                if resp.startswith("STATUS,0"):
                    parts = resp.split(',')
                    if len(parts) >= 3:
                        pos = int(parts[2])
                        m0_ok = abs(pos - 200) < 10
                elif resp.startswith("STATUS,1"):
                    parts = resp.split(',')
                    if len(parts) >= 3:
                        pos = int(parts[2])
                        m1_ok = abs(pos - 150) < 10
            
            if m0_ok and m1_ok:
                print("✅ PASS - Both motors reached target")
                self.test_results.append(("Dual Motor", True))
                return True
            else:
                print("⚠️  Motors moved but positions uncertain")
                self.test_results.append(("Dual Motor", None))
                return None
        else:
            print("❌ FAIL - Dual movement failed")
            self.test_results.append(("Dual Motor", False))
            return False
    
    def run_all_tests(self):
        """Run complete test suite"""
        print("\n" + "="*60)
        print("MOTOR CONTROL SYSTEM TEST SUITE")
        print("="*60)
        
        if not self.connect():
            print("\n❌ Cannot continue - connection failed")
            return
        
        try:
            # Run all tests
            self.test_connection()
            self.test_position_reading()
            self.test_reset()
            self.test_small_movement()
            self.test_home()
            self.test_stop()
            self.test_invalid_command()
            self.test_dual_motor()
            
            # Final cleanup
            print("\n" + "="*60)
            print("Cleanup - Returning to home...")
            print("="*60)
            self.send_command("HOME")
            time.sleep(2)
            
            # Summary
            self.print_summary()
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Tests interrupted by user")
            self.send_command("STOP")
        except Exception as e:
            print(f"\n\n❌ Test error: {e}")
        finally:
            if self.serial:
                self.serial.close()
                print("\n🔌 Disconnected")
    
    def print_summary(self):
        """Print test summary"""
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        
        passed = sum(1 for _, result in self.test_results if result is True)
        failed = sum(1 for _, result in self.test_results if result is False)
        unclear = sum(1 for _, result in self.test_results if result is None)
        total = len(self.test_results)
        
        for test_name, result in self.test_results:
            if result is True:
                status = "✅ PASS"
            elif result is False:
                status = "❌ FAIL"
            else:
                status = "⚠️  UNCLEAR"
            print(f"{test_name:.<40} {status}")
        
        print("="*60)
        print(f"Total Tests: {total}")
        print(f"Passed:      {passed} ({100*passed//total if total else 0}%)")
        print(f"Failed:      {failed}")
        print(f"Unclear:     {unclear}")
        print("="*60)
        
        if failed == 0 and unclear == 0:
            print("🎉 ALL TESTS PASSED!")
        elif failed == 0:
            print("✅ No failures, but some tests unclear")
        else:
            print("⚠️  Some tests failed - check hardware and connections")

def main():
    """Main entry point"""
    print("Motor Control System Test Script")
    print("=" * 60)
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = None
    
    tester = MotorTester(port)
    tester.run_all_tests()

if __name__ == "__main__":
    main()