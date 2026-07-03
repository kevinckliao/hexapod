import math
import time

import mujoco
import mujoco.viewer


MODEL_PATH = "hexapod.xml"
LEGS = ("LF", "RF", "LM", "RM", "LR", "RR")

# Stable pose from the original standing controller.
STAND_POSE = {"coxa": 1.58, "femur": -0.30, "tibia": 1.20}

# Walking parameters. Reduce COXA_SWING or GAIT_HZ if the feet slip.
GAIT_HZ = 0.8
COXA_SWING = 0.28
LIFT_FEMUR = -0.55
LIFT_TIBIA = 1.00
SETTLE_TIME = 2.0
GAIT_RAMP_TIME = 1.0

# The two sets move half a cycle apart, leaving three feet on the ground.
TRIPOD_A = {"LF", "RM", "LR"}


def actuator_ids(model):
    """Cache actuator IDs instead of looking up names every simulation step."""
    return {
        leg: {
            joint: mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{joint}_servo"
            )
            for joint in STAND_POSE
        }
        for leg in LEGS
    }


def set_stand_pose(data, ids):
    for leg in LEGS:
        for joint, angle in STAND_POSE.items():
            data.ctrl[ids[leg][joint]] = angle


def update_tripod_gait(data, ids, walk_time):
    """Set position targets for a smooth, alternating tripod gait."""
    ramp = min(1.0, walk_time / GAIT_RAMP_TIME)
    base_phase = 2.0 * math.pi * GAIT_HZ * walk_time

    for leg in LEGS:
        phase = base_phase + (0.0 if leg in TRIPOD_A else math.pi)

        # First half-cycle: lift and move from back to front.
        # Second half-cycle: keep the foot down and push front to back.
        stride = -math.cos(phase)
        lift = max(0.0, math.sin(phase)) ** 2

        # Right-side leg bodies are rotated 180 degrees in the XML, hence
        # their coxa joint must rotate in the opposite direction.
        forward_sign = -1.0 if leg.startswith("L") else 1.0
        coxa = STAND_POSE["coxa"] + forward_sign * COXA_SWING * ramp * stride
        femur = STAND_POSE["femur"] + (LIFT_FEMUR - STAND_POSE["femur"]) * lift * ramp
        tibia = STAND_POSE["tibia"] + (LIFT_TIBIA - STAND_POSE["tibia"]) * lift * ramp

        data.ctrl[ids[leg]["coxa"]] = coxa
        data.ctrl[ids[leg]["femur"]] = femur
        data.ctrl[ids[leg]["tibia"]] = tibia


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    ids = actuator_ids(model)
    set_stand_pose(data, ids)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter()

            if data.time >= SETTLE_TIME:
                update_tripod_gait(data, ids, data.time - SETTLE_TIME)

            mujoco.mj_step(model, data)
            viewer.sync()

            # Keep wall-clock time synchronized with simulation time.
            remaining = model.opt.timestep - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()


