import numpy as np
from stl import mesh
from scipy.spatial import ConvexHull
import trimesh

def generate_convex_hull(input_stl_path, output_stl_path):
    """
    Generate a collision representation of an STL file using the convex hull method
    and save the resulting mesh as an STL file.

    Parameters:
        input_stl_path (str): Path to the input STL file.
        output_stl_path (str): Path to save the output STL file.
    """
    # Load the STL file
    input_mesh = mesh.Mesh.from_file(input_stl_path)
    
    # Extract the vertices
    vertices = np.unique(input_mesh.vectors.reshape(-1, 3), axis=0)
    
    # Compute the convex hull
    hull = ConvexHull(vertices)
    
    # Get all vertices and faces directly from the ConvexHull object
    hull_faces = hull.simplices
    hull_vertices = vertices  # Use all original vertices for face indexing

    # Construct the output mesh using Trimesh
    output_mesh = trimesh.Trimesh(vertices=hull_vertices, faces=hull_faces)

    # Save the convex hull as an STL file
    output_mesh.export(output_stl_path)
    print(f"Collision representation saved as: {output_stl_path}")

# Example usage
input_stl_path = "/home/apshirazi/Isaac_sim_ws/End_Effectors_Widened/R1/Link_7.stl"
output_stl_path = "/home/apshirazi/Isaac_sim_ws/End_Effectors_Widened/R1/Link_7_col.stl"

generate_convex_hull(input_stl_path, output_stl_path)
