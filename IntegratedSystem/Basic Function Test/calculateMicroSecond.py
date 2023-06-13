from datetime import datetime

# Get the current datetime
current_time = datetime.now()

# Extract the millisecond component
milliseconds = current_time.microsecond // 1000

print("Milliseconds:", milliseconds)