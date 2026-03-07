"""
Debug test to see what data is captured with real hardware
"""

from main import GripperControlSystem
import time

# Create and initialize the system
system = GripperControlSystem()
print('Initializing system...')
system.initialize()

# Start a quick recording session
print('Starting recording...')
system.state.data_manager.start_recording_session('debug_test', system.state)
time.sleep(1)  # Record for 1 second
system.state.data_manager.stop_recording_session()

print('Recording stopped. Check data directory for debug_test files.')

# Clean up
print('Cleaning up...')
system.state.running = False
system.state.hardware.cleanup()
system.state.sensors.cleanup()
print('Done.')
