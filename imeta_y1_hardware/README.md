# imeta_y1_hardware

ROS2 `hardware_interface::SystemInterface` wrapper for the IMeta Y1 arm platform.
This package provides a hardware plugin that integrates the vendor Y1 SDK with ros2_control.

This repository contains a thin adapter that calls into the bundled Y1 SDK and exposes
ros2_control state/command interfaces (skeleton implementation). It is intended to be
used as a system plugin loaded by the ros2_control `controller_manager`.

---

## Repository layout

```
imeta_y1_hardware/
├── CMakeLists.txt
├── package.xml
├── imeta_y1_hardware.xml      # plugin description for ament
├── include/imeta_y1_hardware/
│   └── imeta_y1_hardware.hpp
├── src/
│   └── imeta_y1_hardware.cpp
├── y1_sdk/
│   ├── y1_sdk_interface.h     # local SDK wrapper header
│   └── lib/
│       ├── aarch64/           # Jetson/ARM runtime library (if provided)
│       └── x64/               # x86_64 runtime library
│           └── liby1_sdk_x64.so
└── README.md
```

---

## Purpose

- Provide a ros2_control SystemInterface plugin that wraps the vendor Y1 SDK.
- Initialize the Y1 SDK with parameters from the hardware configuration (CAN interface, URDF path and arm options).
- Offer a place to implement read() and write() operations that exchange state and commands with the physical robot.

Note: This package contains the adapter and vendor SDK wrapper, but the concrete read/write logic and exported command/state
interfaces may need to be implemented/extended to match your robot's exact joint set.

---

## Requirements

- ROS 2 Humble (Ubuntu 22.04)
- ament/cmake build tools
- ros2_control (controller_manager, hardware_interface)
- The Y1 SDK binary for your architecture is expected under `y1_sdk/lib/<arch>/`.

If you plan to build on Jetson, ensure the correct `aarch64` SDK is present. On desktop, place the x64 library.

---

## Build

From the workspace root:

```bash
# build only this package (recommended during development)
colcon build --packages-select imeta_y1_hardware --symlink-install

# or build entire workspace
colcon build --symlink-install

# source the workspace after build
source install/setup.bash
```

If the vendor SDK libraries are not available at build or runtime, you may need to install them
or point LD_LIBRARY_PATH to the `y1_sdk/lib/<arch>` directory.

Example runtime LD_LIBRARY_PATH (bash):

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/your/workspace/src/zwind_common/imeta_y1_hardware/y1_sdk/lib/x64
```

---

## Hardware parameters

The `IMetaY1HW` plugin reads required parameters from the `hardware_parameters` map
provided by the ros2_control hardware configuration (usually in a `ros2_controllers` YAML or robot launch file).
The code expects the following keys (strings):

- `can_interface` (string) — e.g. `can0` or `socketcan0`.
- `urdf_path` (string) — path to the robot URDF (used by the SDK adapter if required).
- `arm_end_type` (int stored as string) — integer code describing the end effector type.
- `enable_arm` (string) — `"true"` or `"false"` to enable/disable arm control.

These keys are accessed via `info.hardware_parameters.at("<key>")`; if a key is missing the plugin will log
an error and return an error from `on_init()`.

### Example ros2_control system configuration snippet

Below is an example `ros2_control` system snippet that loads the plugin and provides the hardware parameters:

```yaml
# my_robot_hardware.yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    # other controller_manager params...

my_hardware:
  ros__parameters:
    plugin: "imeta_y1_hardware::IMetaY1HW"
    can_interface: "can0"
    urdf_path: "/path/to/robot.urdf"
    arm_end_type: "0"
    enable_arm: "true"

# In your robot launch you would load the system with these parameters
```

When using the `controller_manager` you typically provide the above YAML to the `controller_manager` node as parameters so the hardware plugin receives them via `HardwareInfo`.

---

## Runtime / Launch

The plugin is exported with `PLUGINLIB_EXPORT_CLASS` in the `src` code; to run it either:

- Create a launch that starts a `controller_manager` (ros2_control) and sets the `my_hardware` parameters to load this system plugin.
- Or include the plugin in an existing ros2_control hardware configuration.

This package does not provide a full launch that configures controllers; it provides the hardware adapter only.

---

## SDK layout and licensing notes

The vendor SDK wrapper header is at `y1_sdk/y1_sdk_interface.h`. The binary shared objects are placed in
`y1_sdk/lib/<arch>/`. The repository contains a prebuilt `liby1_sdk_x64.so` in `y1_sdk/lib/x64/` for convenience.

Make sure you have permission to distribute or use the provided SDK libraries on your target platform. Replace
with your platform's appropriate SDK if needed.

---

## Implementing read/write

`src/imeta_y1_hardware.cpp` already includes the basic lifecycle hooks. The `read()` and `write()` functions
should be implemented to:

- `read()` — fetch current joint states / sensors from the SDK and populate `pos_states_`, `vel_states_`, etc.
- `write()` — send command values from `pos_commands_` / `vel_commands_` / `tau_commands_` to the SDK or controller.

The current file implements initialization and basic lifecycle handling. If you need help mapping SDK API calls to
ros2_control interfaces, open an issue or request a follow-up and include the SDK header documentation.

---

