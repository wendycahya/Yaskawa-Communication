import time
import math as mt

class VelocityCalculator:
    def __init__(self):
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_time = None

    def update_position(self, x, y, z):
        if self.prev_x is None or self.prev_y is None or self.prev_z is None:
            # If it's the first position measurement, store it and return None as velocity
            self.prev_x = x
            self.prev_y = y
            self.prev_z = z
            self.prev_time = time.time()
            return None, None, None

        # Calculate the distance traveled along each axis
        delta_x = x - self.prev_x
        delta_y = y - self.prev_y
        delta_z = z - self.prev_z

        # Calculate the time elapsed
        current_time = time.time()
        delta_time = current_time - self.prev_time

        # Calculate the velocity components along each axis
        velocity_x = delta_x / delta_time
        velocity_y = delta_y / delta_time
        velocity_z = delta_z / delta_time

        # Update previous positions and time
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_time = current_time

        return velocity_x, velocity_y, velocity_z

    @staticmethod
    def calculate_magnitude_velocity(velocity_x, velocity_y, velocity_z):
        # Calculate the magnitude of velocity using the formula
        magnitude = mt.sqrt(velocity_x ** 2 + velocity_y ** 2 + velocity_z ** 2)
        return magnitude

# Example usage:
velocity_calculator = VelocityCalculator()

# Simulate the object moving along the x, y, and z axes over time
p1 = [353.427, -298.333, -307.424]
p2 = [353.427, -198.333, -307.424]
p3 = [353.427, -98.333, -307.424]
p4 = [353.427, 2.333, -307.424]
p5 = [353.427, 102.333, -307.424]
p6 = [353.427, 202.333, -307.424]

postMove = [p1, p2, p3, p4, p5, p6]
for i in postMove:
    x = i[0]
    y = i[1]
    z = i[2]

    # Calculate velocity
    vx, vy, vz = velocity_calculator.update_position(x, y, z)

    if vx is not None and vy is not None and vz is not None:
        # Calculate the magnitude of velocity
        magnitude_velocity = velocity_calculator.calculate_magnitude_velocity(vx, vy, vz)
        print(f"Time: {time.time()}, X: {x}, Y: {y}, Z: {z}")
        print(f"Velocity: Vx: {vx}, Vy: {vy}, Vz: {vz}")
        print(f"Magnitude Velocity: {magnitude_velocity}")
    else:
        # Print when there is not enough data to calculate velocity
        print(f"Time: {time.time()}, X: {x}, Y: {y}, Z: {z}")
        print("Not enough data points to calculate velocity.")

    time.sleep(1)  # Simulate time passing (in a real application, you'd get the position from your data source)
