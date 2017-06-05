from plyfile import (PlyData, PlyElement, make2d, PlyParseError, PlyProperty)
import numpy as np
import h5py

DATASET_PATH = "ModelNet10_PLY/"

def read_ply_cloud(filename):
    ply_data = PlyData.read(filename)
    points = ply_data['vertex'].data.copy()
    cloud = np.empty([2048, 3])
    for i in range(len(points)):
        point = points[i]
        p = np.array([point[0], point[1], point[2]])
        cloud[i] = p
    return np.array(cloud)

def compute_centroid(cloud):
    points = cloud.shape[0]
    sum_x = np.sum(cloud[:, 0])
    sum_y = np.sum(cloud[:, 1])
    sum_z = np.sum(cloud[:, 2])
    return np.array([sum_x/points, sum_y/points, sum_z/points])

def distance_point_cloud(point, cloud):
    dists = (cloud - point)**2
    dists = np.sum(dists, axis=1)
    dists = np.sqrt(dists)
    return dists

def normalize_unit_sphere(cloud):
    centroid = compute_centroid(cloud)
    dists = distance_point_cloud(centroid, cloud)
    max_dist = np.max(dists)
    normalized_cloud = cloud / max_dist
    return normalized_cloud

def save_h5(h5_filename, data, label, data_dtype='float', label_dtype='uint8'):
    h5_fout = h5py.File(h5_filename)
    h5_fout.create_dataset(
            'data', data=data,
            compression='gzip', compression_opts=4,
            dtype=data_dtype)
    h5_fout.create_dataset(
            'label', data=label,
            compression='gzip', compression_opts=1,
            dtype=label_dtype)
    h5_fout.close()


training_data = []
training_labels = []
test_data = []
test_labels = []

with open(DATASET_PATH + "training_samples.txt") as f:
    lines = f.readlines()
    n_samples = len(lines)
    training_data = np.empty([n_samples, 2048, 3])
    training_labels = np.empty([n_samples, 1])
    for i in range(n_samples):
        splits = lines[i].strip().split(' ')
        data = read_ply_cloud(DATASET_PATH + splits[0])
        training_data[i] = normalize_unit_sphere(data)
        training_labels[i] = int(splits[1])

with open(DATASET_PATH + "test_samples.txt") as f:
    lines = f.readlines()
    n_samples = len(lines)
    test_data = np.empty([n_samples, 2048, 3])
    test_labels = np.empty([n_samples, 1])
    for i in range(n_samples):
        splits = lines[i].strip().split(' ')
        data = read_ply_cloud(DATASET_PATH + splits[0])
        test_data[i] = normalize_unit_sphere(data)
        test_labels[i] = int(splits[1])

test_data = np.array(test_data)
training_data = np.array(training_data)
test_labels = np.array(test_labels)
training_labels = np.array(training_labels)

print(test_data.shape)
print(test_labels.shape)

save_h5("modelnet10_test_normalized.h5", test_data, test_labels)
save_h5("modelnet10_train_normalized.h5", training_data, training_labels)
