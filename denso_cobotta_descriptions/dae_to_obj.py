import os
import pyassimp

from pyassimp.core import AssimpLib

AssimpLib("/Users/ahmed/assimp/bin/libassimp.dylib").load()


def convert_dae_to_obj(dae_path, obj_path):
    """Convert a .dae file to .obj format using pyassimp"""
    try:
        # Load the .dae file
        scene = pyassimp.load(dae_path)

        # Export to .obj format
        pyassimp.export(scene, obj_path, "obj")

        print(f"Successfully converted: {dae_path} -> {obj_path}")
    except Exception as e:
        print(f"Failed to convert {dae_path}: {str(e)}")
    finally:
        # Release the scene resources
        if "scene" in locals():
            pyassimp.release(scene)


def batch_convert_dae_to_obj(directory):
    """Convert all .dae files in a directory to .obj format"""
    if not os.path.exists(directory):
        print(f"Directory does not exist: {directory}")
        return

    # Create 'obj' subdirectory if it doesn't exist
    obj_dir = os.path.join(directory, "obj")
    os.makedirs(obj_dir, exist_ok=True)

    # Find all .dae files in the directory
    dae_files = [f for f in os.listdir(directory) if f.lower().endswith(".dae")]

    if not dae_files:
        print("No .dae files found in the directory")
        return

    print(f"Found {len(dae_files)} .dae files to convert")

    for dae_file in dae_files:
        dae_path = os.path.join(directory, dae_file)
        obj_filename = os.path.splitext(dae_file)[0] + ".obj"
        obj_path = os.path.join(obj_dir, obj_filename)

        convert_dae_to_obj(dae_path, obj_path)


if __name__ == "__main__":
    target_directory = input("Enter the directory path containing .dae files: ").strip()
    batch_convert_dae_to_obj(target_directory)
