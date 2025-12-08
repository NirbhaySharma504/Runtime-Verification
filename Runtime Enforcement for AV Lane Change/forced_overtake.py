# two_car_rear_right_change_smooth.py
# 2 cars in same lane, rear car moves RIGHT lane while continuing forward.
# Smooth throttle ramp and shorter lane-change duration to avoid slowdown-then-shoot effect.

import carla
import time
HOST = "127.0.0.1"
PORT = 2000

# Tweak these for speed/feel
STEP_SLEEP = 0.03            # seconds per update
STRAIGHT_STEPS = 60          # frames before lane change
LANE_CHANGE_STEPS = 40       # fewer steps -> faster lane change
SPACING = 8.0                # distance between rear and front car (meters)
RIGHT_SHIFT = 3.5            # lateral shift for lane change (meters)
FORWARD_ADVANCE = 10.0       # forward movement while lane changing (meters)

# throttle settings
FRONT_THROTTLE = 0.55          # front car steady cruise
REAR_THROTTLE_STRAIGHT = 0.45  # rear normal before change
REAR_THROTTLE_PEAK = 0.95      # rear peak during change (ramped)

def interp_transform(t1, t2, a):
    x = t1.location.x + (t2.location.x - t1.location.x) * a
    y = t1.location.y + (t2.location.y - t1.location.y) * a
    z = t1.location.z + (t2.location.z - t1.location.z) * a
    yaw1 = t1.rotation.yaw
    yaw2 = t2.rotation.yaw
    dy = yaw2 - yaw1
    if dy > 180: dy -= 360
    if dy < -180: dy += 360
    yaw = yaw1 + dy * a
    return carla.Transform(carla.Location(x=x, y=y, z=z),
                           carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0))

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # choose a guaranteed 4-wheel car blueprint
    car_bp = bp_lib.find("vehicle.tesla.model3")

    spawned = []
    try:
        spawns = world.get_map().get_spawn_points()
        if not spawns:
            print("No spawn points found — start CARLA with a map loaded.")
            return

        base = spawns[0]

        # FRONT car: at base spawn
        front_tf = carla.Transform(base.location, base.rotation)
        front = world.try_spawn_actor(car_bp, front_tf)
        if not front:
            print("Failed to spawn front car.")
            return
        spawned.append(front)
        print("Spawned FRONT car id =", front.id)

        # REAR car: behind the front by SPACING meters
        rear_loc = base.location - base.get_forward_vector() * SPACING
        rear_tf = carla.Transform(rear_loc, base.rotation)
        rear = world.try_spawn_actor(car_bp, rear_tf)
        if not rear:
            print("Failed to spawn rear car — destroying front and exiting.")
            front.destroy()
            return
        spawned.append(rear)
        print("Spawned REAR car id =", rear.id)

        # DRIVE STRAIGHT phase
        for _ in range(STRAIGHT_STEPS):
            if front.is_alive:
                front.apply_control(carla.VehicleControl(throttle=FRONT_THROTTLE, steer=0.0))
            if rear.is_alive:
                rear.apply_control(carla.VehicleControl(throttle=REAR_THROTTLE_STRAIGHT, steer=0.0))
            time.sleep(STEP_SLEEP)

        # Compute target transform for rear: forward + right shift
        start_tf = rear.get_transform()
        fwd = start_tf.get_forward_vector()
        right = start_tf.get_right_vector()
        end_location = start_tf.location + carla.Location(
            x = right.x * RIGHT_SHIFT + fwd.x * FORWARD_ADVANCE,
            y = right.y * RIGHT_SHIFT + fwd.y * FORWARD_ADVANCE,
            z = right.z * RIGHT_SHIFT + fwd.z * FORWARD_ADVANCE
        )
        end_tf = carla.Transform(end_location, start_tf.rotation)

        # Lane change with smooth throttle ramp
        for i in range(1, LANE_CHANGE_STEPS + 1):
            a = i / float(LANE_CHANGE_STEPS)
            # transform interpolation
            new_tf = interp_transform(start_tf, end_tf, a)
            try:
                rear.set_transform(new_tf)
            except Exception:
                pass

            # compute ramped throttle (linear from straight -> peak)
            throttle = REAR_THROTTLE_STRAIGHT + (REAR_THROTTLE_PEAK - REAR_THROTTLE_STRAIGHT) * a
            # Apply control each frame (rear uses ramped throttle, front keeps cruising)
            try:
                rear.apply_control(carla.VehicleControl(throttle=throttle, steer=0.0))
            except Exception:
                pass
            if front.is_alive:
                front.apply_control(carla.VehicleControl(throttle=FRONT_THROTTLE, steer=0.0))

            time.sleep(STEP_SLEEP)

        # After lane change, settle rear to a cruise throttle slightly above normal, then back to steady
        for _ in range(50):
            if rear.is_alive:
                rear.apply_control(carla.VehicleControl(throttle=REAR_THROTTLE_STRAIGHT, steer=0.0))
            if front.is_alive:
                front.apply_control(carla.VehicleControl(throttle=FRONT_THROTTLE, steer=0.0))
            time.sleep(STEP_SLEEP)

        # observe
        time.sleep(1.0)

    finally:
        print("Cleaning up — destroying actors...")
        for a in spawned:
            try:
                a.destroy()
            except Exception:
                pass
        print("Done. Exiting.")

if __name__ == "__main__":
    main()
