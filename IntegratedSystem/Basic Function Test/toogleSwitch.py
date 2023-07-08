toggle = False
count = 0

def increment_condition():
    global toggle, count
    toggle = not toggle
    if toggle:
        count += 1

# Example usage
print(count)  # Output: 0

increment_condition()
print(count)  # Output: 1

increment_condition()
print(count)  # Output: 1

increment_condition()
print(count)  # Output: 2