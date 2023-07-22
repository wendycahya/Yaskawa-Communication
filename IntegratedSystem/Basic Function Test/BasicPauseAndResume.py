import threading
import time

def my_function():
    for i in range(10):
        # Check if the loop is paused
        while pause_event.is_set():
            time.sleep(0.1)  # Sleep to reduce CPU usage when paused

        # Perform your processing on each item in the loop
        print(f"Processing item {i}")
        time.sleep(1)  # Simulate some work or processing time

# Create a threading.Event object to control pause and resume
pause_event = threading.Event()
pause_event.set()  # The loop will start in the paused state

# Start the thread that runs the loop
thread = threading.Thread(target=my_function)
thread.start()

# Let the loop run for a few iterations
time.sleep(2)

# Pause the loop
print("Pausing the loop...")
pause_event.set()

# Wait for a few seconds
time.sleep(3)

# Resume the loop
print("Resuming the loop...")
pause_event.clear()

# Wait for the loop to finish
thread.join()

print("Loop finished.")