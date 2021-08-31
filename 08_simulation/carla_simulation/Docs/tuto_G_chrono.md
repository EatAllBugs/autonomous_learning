# Chrono Integration

This guide outlines what Chrono is, how to use it in CARLA, and the limitations involved in the integration.

- [__Project Chrono__](#project-chrono)
- [__Using Chrono on CARLA__](#using-chrono-on-carla)
    - [Configuring the server](#configuring-the-server)
    - [Enabling Chrono physics](#enabling-chrono-physics)
- [__Limitations__](#limitations)

---

## Project Chrono

[Project Chrono](https://projectchrono.org/) is an open-source, multi-physics simulation engine that provides highly realistic vehicle dynamics using a template-based approach. The integration in CARLA allows users to utilize Chrono templates to simulate vehicle dynamics while navigating a map.

---

## Using Chrono on CARLA

To use the Chrono integration, you must first configure the server with a tag on startup and then use the PythonAPI to enable it on a spawned vehicle. Read on for more details.

### Configuring the server

Chrono will only work if the CARLA server is compiled with the Chrono tag.

__In the build from source version of CARLA__, run the following command to start the server:

```sh
make launch ARGS="--chrono"
```

---

### Enabling Chrono physics

Chrono physics is enabled using the `enable_chrono_physics` method available through the [Actor](python_api.md#carlaactor) class. As well as values for substeps and substep delta time, it requires three template files and a base path to locate those files:

- __`base_path`:__ Path of the directory which contains the template files. This is necessary to ensure that auxiliary files referenced from the template files have a common base path from which to search.
- __`vehicle_json`:__ Path of the vehicle template file relative to the `base_path`.
- __`tire_json`:__ Path of the tire template file relative to the `base_path`.
- __`powertrain_json`:__ Path of the powertrain template file relative to the `base_path`.

!!! Important
    Double-check your paths. Incorrect or missing paths can cause Unreal Engine to crash.

There are a variety of example template files for different vehicles available in `Build/chrono-install/share/chrono/data/vehicle`. Read the Project Chrono [documentation](https://api.projectchrono.org/manual_vehicle.html) to find out more about their vehicle examples and how to create templates.

See below for an example of how to enable Chrono physics:

```python
    # Spawn your vehicle
    vehicle = world.spawn_actor(bp, spawn_point)

    # Set the base path
    base_path = "/path/to/carla/Build/chrono-install/share/chrono/data/vehicle/"

    # Set the template files

    vehicle_json = "sedan/vehicle/Sedan_Vehicle.json"
    powertrain_json = "sedan/powertrain/Sedan_SimpleMapPowertrain.json"
    tire_json = "sedan/tire/Sedan_TMeasyTire.json"

    # Enable Chrono physics

    vehicle.enable_chrono_physics(5000, 0.002, vehicle_json, powertrain_json, tire_json, base_path)
```

You can try the Chrono physics integration using the example script `manual_control_chrono.py` found in `PythonAPI/examples`. After running the script, press `Ctrl + o` to enable Chrono.

---

### Limitations

This integration does not support collisions. __When a collision occurs, the vehicle will revert to CARLA default physics.__
