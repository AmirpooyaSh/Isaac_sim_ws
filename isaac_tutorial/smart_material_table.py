from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omni.isaac.core import World
import numpy as np

# Adding mesh to the world (Standalone Format)
from omni.physx.scripts import utils
from omni.isaac.core.utils.stage import add_reference_to_stage

# This is a CuRobo Library that Converts world_model to USD format for Isaac Sim
from curobo.util.usd_helper import UsdHelper
# This is a CuRobo Library to import Different Object Types to the world model
from curobo.geom.types import WorldConfig, Mesh, Cuboid
from curobo.geom.sdf.world import CollisionCheckerType

# CuRobo helper library to create scene !!!
from helper import add_extensions, add_robot_to_scene

cur_dir = "/home/apshirazi/Isaac_sim_ws/"

def add_meshes_to_world():

    Smart_Mat = Mesh(
        name="MatTable_3Level",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path= cur_dir + "smart_table/Smart_Mat_Supply.stl",
        scale=[0.001, 0.001, 0.001],
    )

    world_model = WorldConfig(
        mesh=[Smart_Mat],
        cuboid=[],
        capsule=[],
        cylinder=[],
        sphere=[],
    )

    return world_model


# Giving the World (Meter Scale)
world = World(stage_units_in_meters=1.0)

# Creating "Prim ! (CuRobo)"
stage = world.stage
xform = stage.DefinePrim("/World", "Xform")
stage.SetDefaultPrim(xform)
stage.DefinePrim("/curobo", "Xform")
stage = world.stage

# Adding the default Ground to the World
world.scene.add_default_ground_plane()

# Creating USD Helper (CuRobo Object) to handle objects
usd_help = UsdHelper()

# TiredBear's function to create the list of objects
cuRobo_world = add_meshes_to_world()

# Adding the required Isaac Sim Extensions to the Simulation (Using CuRobo Helper Library)
add_extensions(simulation_app, "False")

# ?????
usd_help.load_stage(world.stage)

# Adding CuRobo World Config to the Stage !!!
usd_help.add_world_to_stage(cuRobo_world, base_frame="/World")

world.reset()

while simulation_app.is_running():
    r=1
    world.step(render=True) # execute one physics step and one rendering step


simulation_app.close()