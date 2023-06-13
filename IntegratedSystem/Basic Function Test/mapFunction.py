def remap(value, from_low, from_high, to_low, to_high):
    # Clamp the value within the from range
    clamped_value = max(from_low, min(value, from_high))

    # Map the clamped value to the to range
    mapped_value = (clamped_value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

    return mapped_value

# Example usage
value = 1200
mapped_value = remap(value, 0, 1500, 0, 800)

print("Mapped value:", mapped_value)