import numpy as np

# shift = -1
# size = 10
# d = 2
# line_points = np.arange(size).reshape(size // d, -1) #
# new_data = np.ones((size, d), dtype=np.float32)
# print(line_points)
#
# line_points[:, :] = np.roll(line_points[:, :], shift, axis=0)
# print(line_points)

# [:, 0] shift only left array
# [:, :] shift all inner numbers with in the arrays

new_data = {}
new_data_pair = np.zeros((2,), dtype=np.float32)
new_data_pair[1] = 3
new_data["test"] = [new_data_pair]
new_data["test"].append(new_data_pair)
print(f"{new_data['test']=}")

new_data_points = new_data["test"]
flipped_np_new_data_points = np.array(new_data_points)
print(f"{flipped_np_new_data_points=}")
