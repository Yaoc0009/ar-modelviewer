import open3d as o3d
import numpy as np
import os

filename = 'assets/gargoyle.txt'
with open(filename, 'r') as f:
    lines = f.readlines()

v = []
v_normal = []
f = []

for line in lines:
    line_split = line.strip().split()
    if line_split[0] == 'Vertex':
        v.append([float(line_split[2]), float(line_split[3]), float(line_split[4])])
        v_normal.append([float(line_split[5].split('(')[-1]), float(line_split[6]), float(line_split[7].split(')')[0])])
    elif line_split[0] == 'Face':
        f.append([int(line_split[2]), int(line_split[3]), int(line_split[4])])

v = np.array(v) # divide by 100000 for gargoyle
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

line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(v),
    lines=o3d.utility.Vector2iVector(e)
)
line_set.paint_uniform_color([0, 0, 0])
o3d.visualization.draw_geometries([he_mesh, line_set])

output_filename = filename.replace('.txt', '.glb')
if not os.path.exists(output_filename):
    o3d.io.write_triangle_mesh(output_filename, mesh)