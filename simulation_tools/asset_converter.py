from omni.isaac.kit import SimulationApp

# Initialize the simulation app
headless = False
simulation_app = SimulationApp({"headless": headless})

# Import necessary modules
from omni.kit.asset_converter import AssetConverterContext, AssetConverter

# Specify paths
mesh_file_path = "/home/apshirazi/Isaac_sim_ws/meshes/MatTable_3Level.stl"  # Replace with your mesh file path
usd_output_path = "/path/to/your/output_file.usd"  # Replace with your desired output path

# Set up the asset converter context
context = AssetConverterContext()
context.ignore_materials = False
context.create_world_as_default_root_prim = True
context.destination_asset_path = usd_output_path

# Perform the conversion
converter = AssetConverter(context)
success = converter.convert(mesh_file_path)

if success:
    print("Conversion succeeded.")
else:
    print("Conversion failed.")

# Close the simulation app
simulation_app.close()