"""Run the deterministic xArm7 pick-and-place baseline."""

import argparse
import json
from pathlib import Path
import time

import mujoco
import mujoco.viewer
from PIL import Image

from mujoco_demo.contracts import PickPlaceTask
from mujoco_demo.pick_place import PickPlaceSimulation


def save_snapshot(simulation: PickPlaceSimulation, path: Path) -> None:
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.35, 0, 0.45]
    camera.distance = 1.2
    camera.azimuth = 145
    camera.elevation = -25
    renderer = mujoco.Renderer(simulation.model, height=720, width=960)
    try:
        renderer.update_scene(simulation.data, camera=camera)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(renderer.render()).save(path)
    finally:
        renderer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="show the MuJoCo viewer in real time")
    parser.add_argument("--screenshots-dir", type=Path, help="save initial and final simulation frames")
    args = parser.parse_args()

    task = PickPlaceTask(object_name="red_cube", target_name="target_zone")
    if args.viewer:
        simulation = PickPlaceSimulation()
        with mujoco.viewer.launch_passive(simulation.model, simulation.data) as viewer:
            simulation.viewer = viewer
            result = simulation.run(task)
            print(json.dumps(result, default=lambda value: value.__dict__, indent=2))
            print("Close the viewer window to exit.")
            while viewer.is_running():
                viewer.sync()
                time.sleep(1 / 60)
    else:
        simulation = PickPlaceSimulation()
        if args.screenshots_dir:
            save_snapshot(simulation, args.screenshots_dir / "initial.png")
        result = simulation.run(task)
        if args.screenshots_dir:
            save_snapshot(simulation, args.screenshots_dir / "final.png")
        print(json.dumps(result, default=lambda value: value.__dict__, indent=2))
    raise SystemExit(0 if result.success else 1)


if __name__ == "__main__":
    main()
