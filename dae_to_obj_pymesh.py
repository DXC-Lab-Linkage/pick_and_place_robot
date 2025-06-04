from pathlib import Path
import pymeshlab


def convert_dae_to_obj(dae_path, obj_path):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(dae_path)

    # Preserve material information
    ms.save_current_mesh(
        obj_path,
    )


dae_path = Path.cwd() / "denso_cobotta_descriptions/cobotta_description"
print(dae_path)
for fname in dae_path.rglob("*.dae"):
    obj_path = fname.parent / fname.stem
    print(obj_path)
    convert_dae_to_obj(dae_path=str(fname), obj_path=str(obj_path) + ".STL")
