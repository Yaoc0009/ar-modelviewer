import open3d as o3d
import numpy as np
import os

filename = 'assets/gargoyle.txt'
with open(filename, 'r') as f:
    lines = f.readlines()

v = []
f = []

for line in lines:
    line_split = line.strip().split()
    if line_split[0] == 'Vertex':
        v.append([float(line_split[2]), float(line_split[3]), float(line_split[4])])
    elif line_split[0] == 'Face':
        f.append([int(line_split[2]), int(line_split[3]), int(line_split[4])])

v = np.array(v)
v_min = np.min(v)
v_max = np.max(v)
v_scale = v_max - v_min
v = (v - v_min) / v_scale
f = np.array(f) - 1
e = []
for face in f:
    face = sorted(face)
    edges = [
        [face[0], face[1]],
        [face[1], face[2]],
        [face[0], face[2]]
    ]
    e.extend(edges)
e = (list(map(list,set(map(tuple, e)))))
e = np.array(e)

print("Number of Vertices: ", len(v))
print("Number of Edges: ", len(e))
print("Number of Faces: ", len(f))

mesh = o3d.geometry.TriangleMesh(
    vertices=o3d.utility.Vector3dVector(v),
    triangles=o3d.utility.Vector3iVector(f)
)
mesh.paint_uniform_color([1, 0.706, 0])
mesh.compute_vertex_normals()
he_mesh = o3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)

mesh_lineset = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(v),
    lines=o3d.utility.Vector2iVector(e)
)
mesh_lineset.paint_uniform_color([0, 0, 0])

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=1, origin=[0, 0, 0])

base_coord = [[x, 0, y] for x in range(-10, 11, 1) for y in range(-10, 11, 1) if x in [-10, 10] or y in [-10, 10]]
base_end = [i for i in range(len(base_coord)) if base_coord[i][0] in [-10, 10] and base_coord[i][2] in [-10, 10]]
base_face = [
    [base_end[0], base_end[1], base_end[2]],
    [base_end[2], base_end[1], base_end[3]],
]
base_edges = [[i, j] for i in range(len(base_coord)) for j in range(i+1, len(base_coord)) if base_coord[i][0] == base_coord[j][0] or base_coord[i][2] == base_coord[j][2]]
base_coord = np.array(base_coord)/5
base_face = np.array(base_face)
base_edges = np.array(base_edges)

base = o3d.geometry.TriangleMesh(
    vertices=o3d.utility.Vector3dVector(base_coord),
    triangles=o3d.utility.Vector3iVector(base_face)
)
base.paint_uniform_color([0.5, 0.5, 0.5])

base_lineset = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(base_coord),
    lines=o3d.utility.Vector2iVector(base_edges)
)
base_lineset.paint_uniform_color([0, 0, 0])

o3d.visualization.draw_geometries([he_mesh, mesh_lineset, mesh_frame, base, base_lineset])

output_filename = filename.replace('.txt', '.glb')
if not os.path.exists(output_filename):
    o3d.io.write_triangle_mesh(output_filename, mesh)