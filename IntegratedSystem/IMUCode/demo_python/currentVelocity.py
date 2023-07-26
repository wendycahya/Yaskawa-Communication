from anrot_module import *

# Given values
initial_velocity = 0.0  # Initial velocity (m/s)
time_step = 0.01  # Time step (s)


gravity = 9.81 * 1000
# Initialize variables
current_velocity = initial_velocity
m_IMU = anrot_module('./config.json')
data = m_IMU.get_module_data(10)


# acceleration_y = 0.3
# acceleration_z = -9.8

while True:
    # Update the velocity using Euler's method
    # Sample IMU data (acceleration in m/s^2)
    acceleration_y = data['acc'][0]['Y']
    #if (acceleration_x <= -)
    print(acceleration_y)
    current_velocity += acceleration_y * time_step
    # Similar updates for acceleration_y and acceleration_z if available

    # Use sensor fusion techniques for better accuracy if available (e.g., Kalman filtering)

    # Process and use the velocity data as needed
    #print(f"Current Velocity: {current_velocity} m/s")

    # Optional: Add a delay or wait for new IMU data before the next iteration
