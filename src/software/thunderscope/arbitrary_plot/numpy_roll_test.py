import numpy as np

shift = -1
size = 10
d = 2
line_points = np.arange(size).reshape(size // d, -1) #
new_data = np.ones((size, d), dtype=np.float32)
print(line_points)

line_points[:, :] = np.roll(line_points[:, :], shift, axis=0)
print(line_points)

# [:, 0] shift only left array
# [:, :] shift all inner numbers with in the arrays