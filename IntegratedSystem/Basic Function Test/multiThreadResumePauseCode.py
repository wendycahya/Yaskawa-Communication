import threading
import time

# Function that represents the work to be done in a separate thread
def do_work():
    global paused, resume_event
    while True:
        while paused:  # Pause the thread if 'paused' is True
            print("Thread paused...")
            resume_event.wait()
            print("Thread resumed...")
        print("Working...")
        time.sleep(1)

# Global variables to control the pause and resume state
paused = False
resume_event = threading.Event()
resume_event.set()  # Set the event to allow the thread to start

# Create and start the worker thread
worker_thread = threading.Thread(target=do_work)
worker_thread.start()

# Pause the thread after 5 seconds
time.sleep(5)
paused = True

# Wait for 3 seconds in the paused state
time.sleep(3)

# Resume the thread
paused = False
resume_event.set()

# Wait for another 5 seconds
time.sleep(5)

# Pause the thread again
paused = True

# Wait for 3 seconds in the paused state
time.sleep(3)

# Resume the thread
paused = False
resume_event.set()

# Wait for the worker thread to finish
worker_thread.join()

print("All threads finished.")