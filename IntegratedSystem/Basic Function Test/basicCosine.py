import numpy as np
from scipy.spatial.distance import cosine

# Example data
data1 = np.array([1, 2, 3, 4, 5])
data2 = np.array([1, 0, 0, 0, 8])

# Calculate cosine similarity
similarity = 1 - cosine(data1, data2)

print(cosine(data1, data2))
# Check similarity
if similarity > 0.8:  # Adjust the threshold as per your requirements
    print("Data is similar.")
else:
    print("Data is not similar.")