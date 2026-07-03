import time
import mujoco
import mujoco.viewer
import msvcrt

# 1. Load the model directly from your XML string
model = mujoco.MjModel.from_xml_path("hexapod.xml")
data = mujoco.MjData(model)

# 2. Define your target values (Note: MuJoCo uses radians for runtime arrays)
target_values = {
    "coxa": 1.58,
    "femur": -0.3,
    "tibia": 1.2
}

# 3. Define the prefixes for all 6 legs
legs = ["LF", "RF", "LM", "RM", "LR", "RR"]

# 4. Loop through and set the control signals
initial_ctrls = {}
for leg in legs:
    for joint_type, target_angle in target_values.items():
        # Construct the exact actuator name from the XML (e.g., "LF_coxa_servo")
        actuator_name = f"{leg}_{joint_type}_servo"

        # Get the internal ID of this actuator
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)

        # Store and set the target angle in the control array (guard if not found)
        if actuator_id >= 0:
            initial_ctrls[actuator_id] = target_angle
            data.ctrl[actuator_id] = target_angle

# 5. Launch the viewer to simulate and visualize the hexapod moving to the target
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        # Console hotkey: press 'r' to reset controls to initial values
        try:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch.lower() == 'r':
                    for aid, val in initial_ctrls.items():
                        data.ctrl[aid] = val
                    print("Reset: restored initial actuator targets")
                # clear any extra buffered keys
                while msvcrt.kbhit():
                    _ = msvcrt.getwch()
        except Exception:
            pass

        time.sleep(model.opt.timestep)