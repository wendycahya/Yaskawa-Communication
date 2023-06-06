count = 0

def increment():
    global count  # Use the global keyword to modify the global variable
    count += 5

print("Before increment:", count)
increment()
print("After increment:", count)
