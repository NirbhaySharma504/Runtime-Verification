import sys
import os
import argparse
import logging
import time

# --------------------------------------------------------------------------------------
# 1. DEFINE PATHS USING KNOWN LOCATIONS (Using raw strings for stability)
# --------------------------------------------------------------------------------------

# Path A: The exact location of the compiled CARLA client file (.egg)
# This resolves the fundamental 'import carla'
EGG_PATH = r"C:\Users\harsh\OneDrive\Desktop\Codes\RE\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"

# Path B: The directory containing the utility scripts (the 'agents' folder)
# This resolves 'from carla.agents.navigation...'
UTILITY_PATH = r'C:\Users\harsh\OneDrive\Desktop\Codes\RE\PythonAPI\carla'

# --------------------------------------------------------------------------------------
# 2. CONFIGURE PATHS
# --------------------------------------------------------------------------------------
try:
    # A) Add the compiled client file (.egg)
    if os.path.exists(EGG_PATH):
        sys.path.append(EGG_PATH)
    else:
        raise FileNotFoundError(f"EGG file not found at: {EGG_PATH}")

    # B) Add the utility folder
    if os.path.isdir(UTILITY_PATH):
        sys.path.append(UTILITY_PATH)
    else:
        raise FileNotFoundError(f"Utility folder not found at: {UTILITY_PATH}")

    print("CARLA Python path configured successfully. Imports enabled.")

except Exception as e:
    print(f"Path Configuration ERROR: {e}")
    sys.exit()

# --------------------------------------------------------------------------------------
# 3. IMPORTS (Will now succeed)
# --------------------------------------------------------------------------------------

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner 

# Now you can use GlobalRoutePlanner in your main function.

def main():
    argparser = argparse.ArgumentParser(description="Simple CARLA Autopilot Example")
    argparser.add_argument('--host', default='127.0.0.1', help='IP of the host server')
    argparser.add_argument('-p', '--port', default=2000, type=int, help='TCP port')
    argparser.add_argument('--timeout', default=10.0, type=float, help='Timeout in seconds')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = None
    original_settings = None
    vehicle = None
    actor_list = [] 

    try:
        # -------------------
        # 1. Connect and Setup
        # -------------------
        print(f"Connecting to CARLA server at {args.host}:{args.port}...")
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        world = client.get_world()
        traffic_manager = client.get_trafficmanager()
        
        # Set to synchronous mode for predictable simulation control
        original_settings = world.get_settings()
        settings = world.get_settings()
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
            print("Switched to synchronous mode.")

        # -------------------
        # 2. Spawn Vehicle and Set Autopilot
        # -------------------
        
        # Get a common vehicle blueprint
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        # Get a safe spawn point
        spawn_point = world.get_map().get_spawn_points()[0]
        
        # Spawn the vehicle
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)
        
        print(f"Spawned vehicle {vehicle.id} at {spawn_point.location}")
        
        # **Enable Autopilot** (Tell the Traffic Manager to control this vehicle)
        vehicle.set_autopilot(True) 
        print("Autopilot enabled.")
        sampling_resolution = 2
        grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
        point_a = spawn_point.location
        point_b = carla.Location(x=-113.903656, y = 14.422489, z = 0.003719)
        route = grp.trace_route(point_a, point_b)

        for waypoint in route:
            world.debug.draw_string(waypoint[0].transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=120.0, persistent_lines=True)
        
        # -------------------
        # 3. Infinite Simulation Loop
        # -------------------
        i = 0
        print("\nStarting infinite simulation loop. Press Ctrl+C to stop.")
        while True:
            world.tick() # Advance the simulation one step
            i += 1
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user (Ctrl+C).")

    except RuntimeError as e:
        print(f"\n[ERROR] CARLA Runtime Error: {e}")

    finally:
        # -------------------
        # 4. Cleanup and Reset
        # -------------------
        if client is not None and original_settings is not None:
            try:
                # Destroy spawned actors
                if actor_list:
                    print(f"Destroying {len(actor_list)} actors...")
                    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

                # Reset world settings to original
                if original_settings.synchronous_mode != world.get_settings().synchronous_mode:
                    world.apply_settings(original_settings)
                    print("Restored original world settings.")

            except Exception as e:
                print(f"Cleanup error: {e}")

        print("Done.")

if __name__ == '__main__':

    main()
