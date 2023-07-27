import pandas as pd

# Read the CSV file into a pandas DataFrame
df = pd.read_csv('dataMeneng.csv')

# Extract the 'ColumnB' data into a NumPy array
column_accX = df['accX'].values
column_accY = df['accY'].values
column_accZ = df['accZ'].values

# Print the array and its data type
print(column_accX, column_accY, column_accZ)