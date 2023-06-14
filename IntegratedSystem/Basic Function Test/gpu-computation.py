import tensorflow as tf

# Check if GPU is available and enabled
if tf.config.list_physical_devices('GPU'):
    print('Computation is being performed on GPU.')
else:
    print('Computation is being performed on CPU.')