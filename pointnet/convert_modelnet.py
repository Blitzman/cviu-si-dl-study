import ntpath
import glob
import os
import numpy as np
from subprocess import call

from plyfile import (PlyData, PlyElement, make2d, PlyParseError, PlyProperty)

n_samples = 2048
leaf_size = 0.005
visualize = 0

def read_off_mesh(filename):
    f = open(filename)
    if 'OFF' != f.readline().strip():
        throw('Not a valid OFF header')
    n_verts, n_faces, n_dontknow = tuple([int(s) for s in f.readline().strip().split(' ')])
    verts = []
    for i_vert in range(n_verts):
        verts.append([float(s) for s in f.readline().strip().split(' ')])
    faces = []
    for i_face in range(n_faces):
        faces.append([int(s) for s in f.readline().strip().split(' ')][1:])
    return np.array(verts), np.array(faces)

def write_ply_mesh(verts, faces, filename):

    vertex = np.zeros(verts.shape[0], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    for i in range(verts.shape[0]):
        vertex[i] = (verts[i][0], verts[i][1], verts[i][2])

    face = np.zeros(faces.shape[0], dtype=[('vertex_indices', 'i4', (3,)), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    for i in range(faces.shape[0]):
        face[i] = ([faces[i][0], faces[i][1], faces[i][2]], 255, 255, 255)

    el_vertex = PlyElement.describe(vertex, 'vertex', comments = ['vertices'])
    el_faces = PlyElement.describe(face, 'face', comments = ['faces'])
    ply_out = PlyData([el_vertex, el_faces], text=True)
    ply_out.write(filename)

def sample_mesh(ply_mesh_filename, pcd_cloud_filename, n_samples, leaf_size, visualize):
    binary = "./pcl_mesh_sampling"
    params = str(ply_mesh_filename + " " +  pcd_cloud_filename + " -n_samples " + str(n_samples) + " -leaf_size " + str(leaf_size) + " -visualize " + str(visualize))
    print(binary + " " + params)
    os.system(binary + " " + params)

def read_pcd_cloud(filename):
    f = open(filename)

    # Skip header
    f.readline()
    # Skip VERSION
    f.readline()
    # Skip FIELDS
    f.readline()
    # Skip SIZE
    f.readline()
    # Skip TYPE
    f.readline()
    # Skip COUNT
    f.readline()
    # Skip WIDTH
    f.readline()
    # Skip HEIGHT
    f.readline()
    # Skip VIEWPOINT
    f.readline()

    n_points = int(f.readline().strip().split(' ')[1])
    print(n_points)

    # Skip DATA
    f.readline()

    points = []
    for point in range(n_points):
        points.append([float(s) for s in f.readline().strip().split(' ')])

    return np.array(points)

def write_ply_cloud(pc, filename):
    vertex = np.zeros(pc.shape[0], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    for i in range(pc.shape[0]):
        vertex[i] = (pc[i][0], pc[i][1], pc[i][2])
    ply_out = PlyData([PlyElement.describe(vertex, 'vertex', comments=['vertices'])])
    ply_out.write(filename)

def get_class_name_labels():
    shape_names_filename = 'shape_names.txt'
    shape_names = [line.rstrip() for line in open(shape_names_filename)]
    name_label = []
    for i in range(len(shape_names)):
        name_label.append((shape_names[i], i))
    return name_label

def process_sample(off_mesh_filename, n_samples, leaf_size, visualize, ply_cloud_filename):
    print('Reading OFF mesh file...')
    vertices, faces = read_off_mesh(off_mesh_filename)
    print('Vertices: ' + str(vertices.shape))
    print('Faces: ' + str(faces.shape))

    print('Writing PLY mesh file...')
    write_ply_mesh(vertices, faces, "tmp/mesh.ply")

    print('Sampling mesh...')
    sample_mesh("tmp/mesh.ply", "tmp/cloud.pcd", n_samples, leaf_size, visualize)

    print('Reading PCD cloud...')
    point_cloud = read_pcd_cloud("tmp/cloud.pcd")

    print('Writing PLY cloud file...')
    write_ply_cloud(point_cloud, ply_cloud_filename)

def path_leaf(path):
    head, tail = ntpath.split(path)
    name = tail or ntpath.basename(head)
    return name.split('.')[0]

print("Getting class names and labels...")
class_name_label = get_class_name_labels()
for name_label in class_name_label:
    print(name_label[0] + " " + str(name_label[1]))

test_files = []
training_files = []

for name_label in class_name_label:

    name = name_label[0]
    label = name_label[1]

    print("Getting training files for " + name)
    class_training_samples = glob.glob("ModelNet10/" + name + "/train/*.off", recursive=True)
    print("Got " + str(len(class_training_samples)) + " samples...")
    print("Getting testing files for " + name)
    class_test_samples = glob.glob("ModelNet10/" + name + "/test/*.off", recursive=True)
    print("Got " + str(len(class_test_samples)) + " samples...")

    for sample in class_training_samples:
        sample_name = path_leaf(sample)
        #process_sample(sample, n_samples, leaf_size, visualize, "ModelNet10_PLY/" + sample_name + ".ply")
        training_files.append((sample_name + ".ply", label))

    for sample in class_test_samples:
        sample_name = path_leaf(sample)
        #process_sample(sample, n_samples, leaf_size, visualize, "ModelNet10_PLY/" + sample_name + ".ply")
        test_files.append((sample_name + ".ply", label))

with open("training_samples.txt", "w") as f:
    for training_file in training_files:
        f.write(training_file[0] + " " + str(training_file[1]) + "\n")

with open("test_samples.txt", "w") as f:
    for test_file in test_files:
        f.write(test_file[0] + " " + str(test_file[1]) + "\n")

