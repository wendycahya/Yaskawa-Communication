import threading
import time

# Global condition variable
condition = threading.Condition()

# Global shared data
data = None

# Function for the reader thread
def reader_thread_function():
    global data, condition
    with condition:
        print("Waiting data sending from thread")
        condition.wait()  # Wait until the writer thread signals
        print("Reader thread received data:", data)
        time.sleep(2)
        print("Sleep 2 finished")
        print("Reader thread received data:", data)

# Function for the writer thread
def writer_thread_function():
    global data, condition
    with condition:
        time.sleep(5)  # Simulate some processing time
        data = "Hello from writer thread"
        condition.notify()  # Notify the reader thread

# Create thread objects
reader_thread = threading.Thread(target=reader_thread_function)
writer_thread = threading.Thread(target=writer_thread_function)

# Start the threads
reader_thread.start()
writer_thread.start()

# Wait for threads to finish
reader_thread.join()
writer_thread.join()