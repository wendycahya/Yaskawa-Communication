import time
import numpy as np
import matplotlib.pyplot as plt

# Function to integrate acceleration data to get velocity
def integrate_acceleration(accel_data, delta_t):
    velocity = np.cumsum(accel_data) * delta_t
    return velocity

# Function to generate synthetic IMU acceleration data
def generate_synthetic_acceleration(t):
    return 9.81 + 2 * np.sin(2 * np.pi * 0.1 * t)  # Add a constant gravity component and a sinusoidal variation

# Real-time plotting function
def plot_realtime_velocity(t, velocity):
    plt.figure()
    plt.plot(t, velocity, 'b-', label='Velocity')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Real-time Velocity Plot')
    plt.legend()
    plt.grid(True)
    plt.show(block=False)

def main():
    delta_t = 0.01  # Time interval between IMU readings (in seconds)
    t = [0.0]  # List to store time values
    velocity = [0.0]  # List to store velocity values

    try:
        while True:
            # Read or generate IMU acceleration data
            accel_data = generate_synthetic_acceleration(t[-1])

            # Integrate the acceleration to get velocity
            new_velocity = integrate_acceleration(accel_data, delta_t)

            # Append new data to lists
            t.append(t[-1] + delta_t)
            velocity.append(new_velocity[-1])

            # Real-time plot
            plot_realtime_velocity(t, velocity)

            # Wait for a short interval before the next reading
            time.sleep(delta_t)

    except KeyboardInterrupt:
        print("Real-time plotting stopped.")

if __name__ == "__main__":
    main()