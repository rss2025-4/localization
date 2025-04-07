#!/usr/bin/env python

import multiprocessing

from libracecar.sandbox import isolate


@isolate
def main():
    import time
    from pathlib import Path

    import better_exceptions
    import jax
    import numpy as np
    from liblocalization import (
        deterministic_motion_tracker,
        particles_model,
    )
    from liblocalization.controllers.particles import particles_params

    from libracecar.test_utils import proc_manager

    jax.config.update("jax_platform_name", "cpu")
    np.set_printoptions(precision=5, suppress=True)

    from localization.particle_filter_from_alan import ExampleSimNode

    # map = "/home/dockeruser/racecar_ws/src/localization/test_map/test_map.yaml"
    map = "/home/alan/6.4200/racecar_simulator/maps/stata_basement.yaml"

    procs = proc_manager.new()

    procs.popen(["rviz2"])
    procs.popen(
        ["emacs"],
        cwd="/home/alan/6.4200/Localization Bags - 04012025/localization_testers",
    )
    procs.ros_node_thread(
        ExampleSimNode,
        # deterministic_motion_tracker,
        particles_model(particles_params(plot_level=10, n_particles=500)),
        # particles_model(
        #     particles_params(plot_level=10, n_particles=500, use_motion_model=False)
        # ),
    )

    time.sleep(1.0)

    procs.ros_launch(
        "racecar_simulator", "localization_simulate.launch.xml", f"map:={map}"
    )

    procs.spin()


if __name__ == "__main__":
    main()
