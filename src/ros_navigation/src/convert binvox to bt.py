import binvox_rw

# Read the binvox file and get the voxel data
with open(bytes('map.binvox', 'utf-8'), 'rb') as f:
    model = binvox_rw.read_as_3d_array(f)

# Save the model as a .bt file
with open(bytes('example.bt', 'utf-8'), 'wb') as f:
    model.write(f)