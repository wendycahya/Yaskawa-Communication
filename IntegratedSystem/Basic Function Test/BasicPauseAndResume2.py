import threading
import time

# Create an Event for pausing, waiting, and resuming the thread
pause_event = threading.Event()
resume_event = threading.Event()

# Function for the while loop in a separate thread
def while_loop_function():
    while True:
        while pause_event.is_set():  # Pause the loop if 'paused' is True
            print("Loop paused...")


        # Perform some work in the loop
        print("Loop executing...")
        time.sleep(1)

# Create the while loop thread
while_loop_thread = threading.Thread(target=while_loop_function)

# Start the while loop thread
while_loop_thread.start()

# Main loop for controlling the pause and resume
while True:
    command = input("Enter 'p' to pause, 'r' to resume, or 'q' to quit: ")

    if command == 'p':
        pause_event.set()  # Pause the while loop
    elif command == 'r':
        pause_event.clear()  # Resume the while loop
        resume_event.set()
    elif command == 'q':
        break  # Quit the program
    else:
        print("Invalid command!")

# Wait for the while loop thread to finish (if you don't want to run indefinitely)
while_loop_thread.join()