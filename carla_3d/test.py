#!/usr/bin/env python3

import math
import random
import time

import carla
import cv2
import numpy as np

camera_image = None


def clamp(value, vmin, vmax):
    return max(vmin, min(value, vmax))


def get_speed_mps(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def camera_callback(image):
    global camera_image
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))
    camera_image = arr[:, :, :3].copy()


def draw_text(img, text, y, color=(255, 255, 255)):
    cv2.putText(img, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2)


def update_spectator(world, vehicle):
    spectator = world.get_spectator()
    tf = vehicle.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    dist_back = 10.0
    height = 4.5
    loc = carla.Location(
        x=tf.location.x - dist_back * math.cos(yaw),
        y=tf.location.y - dist_back * math.sin(yaw),
        z=tf.location.z + height,
    )
    rot = carla.Rotation(pitch=-16.0, yaw=tf.rotation.yaw, roll=0.0)
    spectator.set_transform(carla.Transform(loc, rot))


def choose_spawn(world):
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    best_spawn = None
    best_score = -1

    for sp in spawn_points:
        wp = carla_map.get_waypoint(
            sp.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        if wp is None or wp.is_junction:
            continue

        score = len(wp.next(40.0)) + len(wp.next(80.0))
        if score > best_score:
            best_score = score
            best_spawn = sp

    if best_spawn is None:
        if not spawn_points:
            raise RuntimeError("No spawn point available")
        return spawn_points[0]

    return best_spawn


def waypoint_ahead(world_map, start_location, distance):
    wp = world_map.get_waypoint(
        start_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    if wp is None:
        raise RuntimeError("Waypoint not found")

    remaining = distance
    step = 6.0

    while remaining > 0.1:
        nxt = wp.next(min(step, remaining))
        if not nxt:
            break
        wp = nxt[0]
        remaining -= min(step, remaining)

    return wp


def spawn_ego(world, blueprint_library, spawn_tf):
    candidates = [
        "vehicle.tesla.model3",
        "vehicle.lincoln.mkz_2020",
        "vehicle.audi.etron",
    ]

    bp = None
    for name in candidates:
        matches = blueprint_library.filter(name)
        if matches:
            bp = matches[0]
            break

    if bp is None:
        bp = random.choice(blueprint_library.filter("vehicle.*"))

    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "ego")

    return world.spawn_actor(bp, spawn_tf)


def spawn_camera(world, ego, blueprint_library):
    cam_bp = blueprint_library.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", "1280")
    cam_bp.set_attribute("image_size_y", "720")
    cam_bp.set_attribute("fov", "100")

    cam_tf = carla.Transform(
        carla.Location(x=-6.5, z=2.8),
        carla.Rotation(pitch=-10.0)
    )

    cam = world.spawn_actor(cam_bp, cam_tf, attach_to=ego)
    cam.listen(camera_callback)
    return cam


def spawn_background_traffic(world, tm, ego_spawn, count=25):
    blueprints = [
        bp for bp in world.get_blueprint_library().filter("vehicle.*")
        if bp.has_attribute("number_of_wheels")
    ]
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    actors = []

    for sp in spawn_points:
        if len(actors) >= count:
            break

        if sp.location.distance(ego_spawn.location) < 25.0:
            continue

        bp = random.choice(blueprints)

        if bp.has_attribute("color"):
            bp.set_attribute(
                "color",
                random.choice(bp.get_attribute("color").recommended_values)
            )

        if bp.has_attribute("driver_id"):
            bp.set_attribute(
                "driver_id",
                random.choice(bp.get_attribute("driver_id").recommended_values)
            )

        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle is None:
            continue

        vehicle.set_autopilot(True, tm.get_port())
        tm.vehicle_percentage_speed_difference(vehicle, random.choice([0, 5, 10, 15]))
        actors.append(vehicle)

    return actors


def spawn_test_vehicles(world, blueprint_library, ego_spawn):
    world_map = world.get_map()
    actors = []
    control = {
        "vehicles": [],
        "cycle_seconds": 10.0,
        "move_seconds": 5.0
    }

    distances = [32.0, 46.0, 60.0]

    for i, d in enumerate(distances):
        wp = waypoint_ahead(world_map, ego_spawn.location, d)

        bp = blueprint_library.filter("vehicle.audi.a2")
        if bp:
            bp = bp[0]
        else:
            bp = random.choice(blueprint_library.filter("vehicle.*"))

        tf = wp.transform
        tf.location.z += 0.5

        v = world.try_spawn_actor(bp, tf)
        if v is not None:
            actors.append(v)
            control["vehicles"].append({
                "actor": v,
                "speed_mps": (10.0 if i == 0 else 6.0) / 3.6
            })

    if not actors:
        raise RuntimeError("Unable to create test vehicles")

    return actors, control


def control_test_vehicles(control, elapsed):
    cycle = control.get("cycle_seconds", 10.0)
    move_seconds = control.get("move_seconds", 5.0)
    moving = (elapsed % cycle) < move_seconds

    for item in control["vehicles"]:
        actor = item["actor"]
        if not actor.is_alive:
            continue

        actor.disable_constant_velocity()

        if moving:
            actor.apply_control(
                carla.VehicleControl(throttle=0.34, brake=0.0, steer=0.0)
            )
        else:
            actor.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False)
            )


def same_lane(world, ego, other):
    carla_map = world.get_map()

    ego_wp = carla_map.get_waypoint(
        ego.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    oth_wp = carla_map.get_waypoint(
        other.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    if ego_wp is None or oth_wp is None:
        return False

    return ego_wp.road_id == oth_wp.road_id and ego_wp.lane_id == oth_wp.lane_id


def find_front_vehicle(world, ego_vehicle, max_distance=90.0):
    ego_tf = ego_vehicle.get_transform()
    ego_loc = ego_tf.location
    forward = ego_tf.get_forward_vector()

    closest = None
    min_dist = max_distance

    for vehicle in world.get_actors().filter("vehicle.*"):
        if vehicle.id == ego_vehicle.id:
            continue

        if not same_lane(world, ego_vehicle, vehicle):
            continue

        loc = vehicle.get_transform().location
        dx = loc.x - ego_loc.x
        dy = loc.y - ego_loc.y
        dz = loc.z - ego_loc.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist > max_distance:
            continue

        dot = forward.x * dx + forward.y * dy
        if dot <= 0.0:
            continue

        if dist < min_dist:
            min_dist = dist
            closest = vehicle

    return closest, min_dist


def compute_ttc(distance, ego_speed, front_speed):
    rel_speed = ego_speed - front_speed
    if rel_speed <= 0.1:
        return float("inf"), rel_speed
    return distance / rel_speed, rel_speed


def speed_controller(cruise_speed_mps, ego_speed, front_detected, front_distance, front_speed):
    emergency_distance = 7.0
    close_distance = 12.0
    follow_distance = 22.0

    if not front_detected:
        err = cruise_speed_mps - ego_speed
        return cruise_speed_mps, 1.2 * err, float("inf"), 0.0, "CRUISE"

    ttc, rel_speed = compute_ttc(front_distance, ego_speed, front_speed)

    if front_distance < emergency_distance or ttc < 1.2:
        return 0.0, -6.0, ttc, rel_speed, "EMERGENCY_BRAKE"

    if front_distance < close_distance or ttc < 2.3:
        target = min(front_speed, 2.0 / 3.6)
        return target, -3.8, ttc, rel_speed, "STRONG_BRAKE"

    if front_distance < follow_distance or ttc < 4.5:
        target = min(cruise_speed_mps, front_speed + 1.0 / 3.6)
        err = target - ego_speed
        a_ref = 0.9 * err - 0.18 * max(0.0, rel_speed)
        return target, a_ref, ttc, rel_speed, "FOLLOW"

    err = cruise_speed_mps - ego_speed
    return cruise_speed_mps, 1.0 * err, ttc, rel_speed, "CRUISE"


def accel_to_control(a_ref):
    a_acc_max = 3.0
    a_brake_max = 6.0

    if a_ref >= 0.0:
        throttle = clamp(a_ref / a_acc_max, 0.0, 0.85)
        brake = 0.0
    else:
        throttle = 0.0
        brake = clamp((-a_ref) / a_brake_max, 0.0, 1.0)

    return throttle, brake


def compute_lane_steer(world, vehicle, lookahead=9.0, gain=1.35):
    world_map = world.get_map()
    tf = vehicle.get_transform()
    loc = tf.location
    yaw = math.radians(tf.rotation.yaw)

    wp = world_map.get_waypoint(
        loc,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    if wp is None:
        return 0.0

    nxt = wp.next(lookahead)
    if not nxt:
        return 0.0

    target = nxt[0].transform.location
    dx = target.x - loc.x
    dy = target.y - loc.y
    angle = math.atan2(dy, dx)

    error = angle - yaw
    while error > math.pi:
        error -= 2 * math.pi
    while error < -math.pi:
        error += 2 * math.pi

    return clamp(gain * error, -1.0, 1.0)


def main():
    window_name = "Speed Regulation"

    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)

    world = client.get_world()
    original_settings = world.get_settings()
    blueprint_library = world.get_blueprint_library()
    tm = client.get_trafficmanager(8000)

    ego = None
    camera = None
    npc_actors = []
    test_actors = []

    try:
        print("Connecting to CARLA...")

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        tm.set_synchronous_mode(True)
        tm.global_percentage_speed_difference(10.0)
        tm.set_global_distance_to_leading_vehicle(2.5)
        world.set_weather(carla.WeatherParameters.CloudySunset)

        ego_spawn = choose_spawn(world)
        ego = spawn_ego(world, blueprint_library, ego_spawn)
        print("Ego vehicle created")

        camera = spawn_camera(world, ego, blueprint_library)
        npc_actors = spawn_background_traffic(world, tm, ego_spawn, count=25)
        print(f"Background traffic: {len(npc_actors)} vehicles")

        test_actors, test_control = spawn_test_vehicles(world, blueprint_library, ego_spawn)
        print(f"Test vehicles: {len(test_actors)}")

        cruise_speed_kmh = 35.0
        cruise_speed_mps = cruise_speed_kmh / 3.6

        prev_speed = 0.0
        start_time = time.time()
        last_print = time.time()

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        print("Simulation running. Press q to quit.")

        while True:
            world.tick()
            elapsed = time.time() - start_time
            control_test_vehicles(test_control, elapsed)

            ego_speed = get_speed_mps(ego)
            measured_acc = (ego_speed - prev_speed) / settings.fixed_delta_seconds
            steer = compute_lane_steer(world, ego)

            front_vehicle, front_distance = find_front_vehicle(world, ego, max_distance=90.0)
            front_detected = front_vehicle is not None
            front_speed = get_speed_mps(front_vehicle) if front_detected else 0.0

            target_speed, a_ref, ttc, rel_speed, mode = speed_controller(
                cruise_speed_mps,
                ego_speed,
                front_detected,
                front_distance if front_detected else float("inf"),
                front_speed,
            )

            throttle, brake = accel_to_control(a_ref)

            ego.apply_control(
                carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=steer
                )
            )

            update_spectator(world, ego)

            if time.time() - last_print >= 0.5:
                ttc_txt = "INF" if math.isinf(ttc) else f"{ttc:.2f}s"
                dist_txt = "NONE" if not front_detected else f"{front_distance:.1f}m"
                front_txt = "NONE" if not front_detected else f"{front_speed*3.6:.1f}km/h"

                print(
                    f"Mode={mode:<16} | Ego={ego_speed*3.6:5.1f} km/h | "
                    f"Target={target_speed*3.6:5.1f} km/h | "
                    f"Front={front_txt:<10} | Dist={dist_txt:<8} | "
                    f"TTC={ttc_txt:<7} | Throttle={throttle:.2f} Brake={brake:.2f}"
                )
                last_print = time.time()

            if camera_image is not None:
                display = camera_image.copy()
                ttc_txt = "INF" if math.isinf(ttc) else f"{ttc:.2f} s"

                draw_text(display, "Intelligent Speed Regulation", 40, (0, 255, 255))
                draw_text(display, f"Mode: {mode}", 90, (0, 255, 255))
                draw_text(display, f"Vehicle speed: {ego_speed*3.6:.1f} km/h", 140, (0, 255, 0))
                draw_text(display, f"Target speed: {target_speed*3.6:.1f} km/h", 190, (255, 255, 0))
                draw_text(display, f"Measured acceleration: {measured_acc:.2f} m/s^2", 240, (180, 255, 180))
                draw_text(display, f"Throttle command: {throttle:.2f}", 290, (255, 255, 255))
                draw_text(display, f"Brake command: {brake:.2f}", 340, (255, 255, 255))

                if front_detected:
                    draw_text(display, f"Front distance: {front_distance:.1f} m", 400, (255, 220, 0))
                    draw_text(display, f"Front vehicle speed: {front_speed*3.6:.1f} km/h", 450, (0, 255, 255))
                    draw_text(display, f"TTC: {ttc_txt}", 500, (0, 120, 255))
                else:
                    draw_text(display, "Front vehicle: none", 400, (255, 220, 0))
                    draw_text(display, "TTC: INF", 450, (0, 120, 255))

                cv2.imshow(window_name, display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            prev_speed = ego_speed

    except KeyboardInterrupt:
        print("\nStop requested")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Cleaning up...")

        if camera is not None:
            camera.stop()
            camera.destroy()

        for actor in test_actors:
            if actor is not None and actor.is_alive:
                actor.destroy()

        for actor in npc_actors:
            if actor is not None and actor.is_alive:
                actor.destroy()

        if ego is not None and ego.is_alive:
            ego.destroy()

        cv2.destroyAllWindows()
        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)

        print("Program finished")


if __name__ == "__main__":
    main()
