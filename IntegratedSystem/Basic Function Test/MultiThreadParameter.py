import threading
import time

# Function that represents the work to be done in a separate thread
def do_work(parameter):
    # Perform the calculation using the given parameter
    result = parameter * 2
    print(f"Result for parameter {parameter}: {result}")
    time.sleep(1)

# Number of parameters to calculate
num_parameters = 5

# Create and start multiple worker threads
threads = []
for i in range(num_parameters):
    parameter = i + 1
    thread = threading.Thread(target=do_work, args=(parameter,))
    threads.append(thread)
    thread.start()

# Wait for all threads to finish
for thread in threads:
    thread.join()

print("All threads finished.")