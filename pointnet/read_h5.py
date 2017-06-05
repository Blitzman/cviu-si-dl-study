import numpy as np
import h5py
from plyfile import (PlyData, PlyElement, make2d, PlyParseError, PlyProperty)

def export_ply(pc, filename):
    vertex = np.zeros(pc.shape[0], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    for i in range(pc.shape[0]):
        vertex[i] = (pc[i][0], pc[i][1], pc[i][2])
    ply_out = PlyData([PlyElement.describe(vertex, 'vertex', comments=['vertices'])])
    ply_out.write(filename)

def load_h5(h5_filename):
    f = h5py.File(h5_filename)
    data = f['data'][:]
    label = f['label'][:]
    return (data, label)

dataset = load_h5('ply_data_test0.h5')

count = 0
for point_cloud in dataset[0]:
    export_ply(point_cloud, 'clouds/cloud_' + str(count) + '.ply')
    count += 1 

print(dataset[0].shape)
print(dataset[0][0].shape)
print(dataset[1].shape)
