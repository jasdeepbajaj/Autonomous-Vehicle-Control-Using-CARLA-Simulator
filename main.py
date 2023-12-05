# Import necessary libraries and modules
import numpy as np
from global_route_planner import GlobalRoutePlanner
from longitudinal_controller import PIDLongitudinalController
from lateral_controller import PurePursuitController
from utils import control_signal, find_dist_veh
import carla


# Main function
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(40.0)
    world = client.load_world('Town01')

    amap = world.get_map()
    sampling_resolution = 0.5
    grp = GlobalRoutePlanner(amap, sampling_resolution)

    spawn_points = world.get_map().get_spawn_points()


    start = np.random.choice(spawn_points)
    end = np.random.choice(spawn_points)

    a = carla.Location(start.location)
    b = carla.Location(end.location)

    w1 = grp.trace_route(a, b) 

    world.debug.draw_point(a,color=carla.Color(r=255, g=0, b=0),size=0.4 ,life_time=120.0)
    world.debug.draw_point(b,color=carla.Color(r=255, g=0, b=0),size=0.4 ,life_time=120.0)


    wps=[]

    for i in range(len(w1)):
        wps.append(w1[i][0])
        world.debug.draw_point(w1[i][0].transform.location,color=carla.Color(r=255, g=0, b=0),size=0.05 ,life_time=120.0)

    waypoint_list = []

    for i, wp in enumerate(wps):
        waypoint_list.insert(i, (wp.transform.location.x, wp.transform.location.y)) 
        
        

    spawnPoint=start
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.find('vehicle.lincoln.mkz_2020')
    vehicle = world.spawn_actor(bp, spawnPoint)


    L = 2.5
    Kdd = 1.4

    long_controller = PIDLongitudinalController(vehicle, K_P = 1.0 , K_D = 0.0, K_I = 0.75 , dt = 1.0 / 10.0 )
    lat_controller = PurePursuitController(L, Kdd)

    speed=45



    i=0
    target=wps[0]
    past_steering = vehicle.get_control().steer
    while True:

        spectator = world.get_spectator() 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
        spectator.set_transform(transform) 

        vehicle_loc= vehicle.get_location()
        distance_v =find_dist_veh(vehicle_loc,target)
        
        (control, R) = control_signal(vehicle, waypoint_list, long_controller, lat_controller, speed, wps, past_steering) #new

        # print(R) #new
        past_steering = control.steer       
        
        
        vehicle.apply_control(control)
        
        
        if i==(len(wps)-1):
            print("last waypoint reached")
            break 
        
        
        if (distance_v>3.5):
            control,R = control_signal(vehicle, waypoint_list, long_controller, lat_controller, speed, target, past_steering)
            past_steering = control.steer 
            i=i+1
            target=wps[i]
            

    control,R = control_signal(vehicle, waypoint_list, long_controller, lat_controller, 0, target, past_steering)
    past_steering = control.steer 
    vehicle.apply_control(control)
    vehicle.destroy()


if __name__ == '__main__':
    main()