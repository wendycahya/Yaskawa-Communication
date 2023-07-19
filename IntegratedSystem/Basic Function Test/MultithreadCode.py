import threading

class DataReaderThread(threading.Thread):
    def __init__(self, data_list):
        threading.Thread.__init__(self)
        self.data_list = data_list
        self.data_read = None
        self.lock = threading.Lock()

    def run(self):
        # Simulate reading data
        # In this example, we simply assign the data list to the data_read variable
        with self.lock:
            self.data_read = self.data_list

# Create data list
data = [1, 2, 3, 4, 5]

# Create thread object
reader_thread = DataReaderThread(data)

# Start the thread
reader_thread.start()

# Wait for the thread to complete
reader_thread.join()

# Access the read data from the main program
with reader_thread.lock:
    read_data = reader_thread.data_read

# Process or use the read data
print("Read data:", read_data)