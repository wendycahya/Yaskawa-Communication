import csv

def calculate_mape(actual, predicted):
    errors = []
    for i in range(len(actual)):
        if actual[i] != 0:
            error = abs((actual[i] - predicted[i]) / actual[i])
            errors.append(error)
    mape = sum(errors) / len(errors) * 100
    return mape

# Read data from CSV file
actual_values = []
predicted_values = []
with open('data.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header row if present
    for row in reader:
        actual_values.append(float(row[0]))
        predicted_values.append(float(row[1]))

# Calculate MAPE
mape = calculate_mape(actual_values, predicted_values)
print(f"The MAPE is: {mape:.2f}%")