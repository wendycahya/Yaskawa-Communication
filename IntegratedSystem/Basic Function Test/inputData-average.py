data = []  # List to store the input data

# Collect 10 input values
while len(data) < 10:
    value = float(input("Enter a data value: "))  # Assuming input as float
    data.append(value)

# Calculate the average
average = sum(data) / len(data)

print("Average:", average)