import pandas as pd

# Read the CSV file into a DataFrame
df = pd.read_csv('0609-SSMNewDemo.csv')

# Initialize variables
window_size = 10
data_window = []
#information
write_file = "0612-SSMNewDemo-distance.csv"

# Process each row in the DataFrame
with open(write_file, "wt", encoding="utf-8") as output:
    for index, row in df.iterrows():
        # Access the column containing the sensor data
        data = row['d']

        # Add the data to the window
        data_window.append(data)

        # If the window is full, calculate the average
        if len(data_window) == window_size:
            # Calculate the average
            average = sum(data_window) / window_size

            # Print the average
            print("Real-time average:", average)
            output.write(str(index) + ',' + str(average) + '\n')
            # Remove the oldest data from the window
            data_window.pop(0)
    print("SUCCESS RECORD ", index, " !!!")