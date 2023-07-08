import threading
import time

# Function for Thread 1
def thread_1_function():
    print("Thread 1 started")
    while True:
        print("Thread 1 is running for object detection")
        time.sleep(1)

# Function for Thread 2
def thread_2_function():
    print("Thread 2 started")
    while True:
        print("Thread 2 is running for robot movement")
        time.sleep(1)

# Function for Thread 3
def thread_3_function():
    print("Thread 3 started")
    while True:
        print("Thread 3 is running for conveyor movement")
        time.sleep(1)


# Create Thread objects
thread_1 = threading.Thread(target=thread_1_function)
thread_2 = threading.Thread(target=thread_2_function)
thread_3 = threading.Thread(target=thread_3_function)

# Start the threads
thread_1.start()
thread_2.start()
thread_3.start()

# Wait for threads to finish (which will never happen in this example)
thread_1.join()
thread_2.join()
thread_3.join()