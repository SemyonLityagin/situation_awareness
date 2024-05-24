import json, math
import numpy as np
from random import randrange


DRONE_CAMERA_ANGEL = 45
DRONE_CAMERA_ANGEL_RAD = np.pi*DRONE_CAMERA_ANGEL/180

CAMERA_ANGEL = 30
CAMERA_ANGEL_RAD = np.pi*CAMERA_ANGEL/180

TYPE_CONF_LONG_ZONE = [30,40]
TYPE_CONF_MIDDLE_ZONE = [40,65]
TYPE_CONF_CLOSE_ZONE = [65,90]

CLASS_CONF_MIDDLE_ZONE = 30
CLASS_CONF_CLOSE_ZONE = 50

DRONE_CAMERA_LONG_ZONE = 1000
DRONE_CAMERA_MIDDLE_ZONE = 500
DRONE_CAMERA_CLOSE_ZONE = 100

CAMERA_LONG_ZONE = 100
CAMERA_MIDDLE_ZONE = 50
CAMERA_CLOSE_ZONE = 30

RADAR_LONG_ZONE = 300
RADAR_MIDDLE_ZONE = 150
RADAR_CLOSE_ZONE = 50

CLOSE_ANGLE_NOISE_MEAN = 0.0
MIDDLE_ANGLE_NOISE_MEAN = 0.0
LONG_ANGLE_NOISE_MEAN = 0.0
CLOSE_ANGLE_NOISE_DISP = 0.1
MIDDLE_ANGLE_NOISE_DISP = 0.2
LONG_ANGLE_NOISE_DISP = 0.45

DRONE_CAMERA_DIST_LONG_ZONE_MEAN = 0
DRONE_CAMERA_DIST_MIDDLE_ZONE_MEAN = 0
DRONE_CAMERA_DIST_CLOSE_ZONE_MEAN = 0
DRONE_CAMERA_DIST_LONG_ZONE_DISP = 12
DRONE_CAMERA_DIST_MIDDLE_ZONE_DISP = 8
DRONE_CAMERA_DIST_CLOSE_ZONE_DISP = 5

CAMERA_DIST_LONG_ZONE_MEAN = 0
CAMERA_DIST_MIDDLE_ZONE_MEAN = 0
CAMERA_DIST_CLOSE_ZONE_MEAN = 0
CAMERA_DIST_LONG_ZONE_DISP = 5
CAMERA_DIST_MIDDLE_ZONE_DISP = 3
CAMERA_DIST_CLOSE_ZONE_DISP = 1

RADAR_DIST_LONG_ZONE_MEAN = 0
RADAR_DIST_MIDDLE_ZONE_MEAN = 0
RADAR_DIST_CLOSE_ZONE_MEAN = 0
RADAR_DIST_LONG_ZONE_DISP = 8
RADAR_DIST_MIDDLE_ZONE_DISP = 5
RADAR_DIST_CLOSE_ZONE_DISP = 3

sensor_delta_time = {
        "radar": 2,
        "camera": 1,
        "comm": 40,
        "local": 2,
    }

# т.к. индексы итераций сценария с 0, откатим до 0-го отчета
sensor_timers = {
        "radar": 2-1,
        "camera": 1-1,
        "comm": 40-1,
        "local": 2-1,
    }

sensor_counter = {
        "radar": 0,
        "camera": 0,
        "comm": 0,
        "local": 0,
    }

server_data = {
        "radar": {},
        "camera": {},
        "comm": {},
        "local": {},
    }

def get_triangle_angles(dist, obj_coords, rotate_angle, sensor_angle):
    hight = dist*np.tan(sensor_angle)
    left = [dist, hight]
    right = [dist, -hight]

    #поворот
    left = [
        left[0]*np.cos(rotate_angle) - left[1]*np.sin(rotate_angle),
        left[0]*np.sin(rotate_angle) + left[1]*np.cos(rotate_angle),
    ]
    right = [
        right[0]*np.cos(rotate_angle) - right[1]*np.sin(rotate_angle),
        right[0]*np.sin(rotate_angle) + right[1]*np.cos(rotate_angle),
    ]

    #сдвиг
    left = [
        obj_coords[0] + left[0],
        obj_coords[1] + left[1],
    ]

    right = [
        obj_coords[0] + right[0],
        obj_coords[1] + right[1],
    ]
    #print(left, right)
    return left, right

def obj_in_triangle(point1, point2, point3, obj_coord):
    x1,y1 = point1
    x2,y2 = point2
    x3,y3 = point3
    x0, y0 = obj_coord
    sign1 = np.sign((x1 - x0) * (y2 - y1) - (x2 - x1) * (y1 - y0))
    sign2 = np.sign((x2 - x0) * (y3 - y2) - (x3 - x2) * (y2 - y0))
    sign3 = np.sign((x3 - x0) * (y1 - y3) - (x1 - x3) * (y3 - y0))
    
    return (sign1 == sign2) and (sign1 == sign3)

if __name__ == "__main__":
    with open("plan_scenario.json", "r") as f:
        scenario = json.loads(f.read())

    for idx, time_step in enumerate(scenario["iterations"]):
        stack_to_model = time_step["enemy"] + time_step["ally"]
        t = time_step["t"]

        all_world_obj = stack_to_model.copy()

        for i in range(len(stack_to_model)):
            obj = stack_to_model.pop()
            obj_id = obj["obj_id"]
            obj_agent_name = obj["agent_name"]
            obj_evkl_vel = obj["velocity_evkl"]
            obj_polar_vel = obj["velocity_polar"]
            obj_evkl_coords = obj["coords_evkl"]
            obj_polar_coords = obj["coords_polar"]
            obj_type = obj["type"]
            obj_class = obj["class"]
            obj_friendly = obj["friendly"]
            if obj_evkl_vel[0] == 0 and obj_evkl_vel[1] == 0:
                obj_orientation = 0.0
            else:
                obj_orientation = np.sign(obj_evkl_vel[1])*np.arccos(obj_evkl_vel[0]/(math.sqrt(obj_evkl_vel[0]**2 + obj_evkl_vel[1]**2)))

            for sensor_type in sensor_timers.keys():
                if sensor_timers[sensor_type] == idx:
                    if sensor_type == "radar":
                        server_sensor_data = {
                                "t": t,
                                "obj_id": obj_id,
                                "agent_name": obj_agent_name,
                                "detections": []
                            }

                        for world_obj in all_world_obj:
                            if world_obj["agent_name"] == obj_agent_name:
                                continue

                            world_obj_id = world_obj["obj_id"]
                            world_obj_agent_name = world_obj["agent_name"]
                            world_obj_evkl_vel = world_obj["velocity_evkl"]
                            world_obj_polar_vel = world_obj["velocity_polar"]
                            world_obj_evkl_coords = world_obj["coords_evkl"]
                            world_obj_polar_coords = world_obj["coords_polar"]
                            world_obj_type = world_obj["type"]
                            world_obj_class = world_obj["class"]
                            world_obj_friendly = world_obj["friendly"]

                            distance = np.sqrt((world_obj_evkl_coords[0]-obj_evkl_coords[0])**2 + (world_obj_evkl_coords[1]-obj_evkl_coords[1])**2)

                            coords_noise = [0,0]
                            if distance > RADAR_LONG_ZONE:
                                continue
                            elif distance > RADAR_MIDDLE_ZONE:
                                coords_noise[0] = np.random.normal(RADAR_DIST_LONG_ZONE_MEAN, RADAR_DIST_LONG_ZONE_DISP)
                                coords_noise[1] = np.pi * np.random.normal(LONG_ANGLE_NOISE_MEAN, LONG_ANGLE_NOISE_DISP) / 180
                                quality_type = np.random.randint(TYPE_CONF_LONG_ZONE[0], TYPE_CONF_LONG_ZONE[1])/100
                                quality_class = -1.0
                                world_obj_class = -1
                            elif distance > RADAR_CLOSE_ZONE:
                                coords_noise[0] = np.random.normal(RADAR_DIST_MIDDLE_ZONE_MEAN, RADAR_DIST_MIDDLE_ZONE_DISP)
                                coords_noise[1] = np.pi * np.random.normal(MIDDLE_ANGLE_NOISE_MEAN, MIDDLE_ANGLE_NOISE_DISP) / 180
                                quality_type = np.random.randint(TYPE_CONF_MIDDLE_ZONE[0], TYPE_CONF_MIDDLE_ZONE[1])/100
                                quality_class = np.random.randint(CLASS_CONF_MIDDLE_ZONE, int(quality_type*100))/100
                            elif distance > 0:
                                coords_noise[0] = np.random.normal(RADAR_DIST_CLOSE_ZONE_MEAN, RADAR_DIST_CLOSE_ZONE_DISP)
                                coords_noise[1] = np.pi * np.random.normal(CLOSE_ANGLE_NOISE_MEAN, CLOSE_ANGLE_NOISE_DISP) / 180
                                quality_type = np.random.randint(TYPE_CONF_CLOSE_ZONE[0], TYPE_CONF_CLOSE_ZONE[1])/100
                                quality_class = np.random.randint(CLASS_CONF_CLOSE_ZONE, int(quality_type*100))/100

                            detection = {
                                    "category": 1,
                                    "obj_id": str(world_obj_id),
                                    "class": world_obj_class,
                                    "type": world_obj_type,
                                    "classConf": quality_class,
                                    "typeConf": quality_type,
                                    "distance":  -1.0,
                                    "azimuth":  world_obj_polar_coords[1] + coords_noise[1],
                                    "friendly": world_obj_friendly
                                }
                            
                            server_sensor_data["detections"].append(detection)  
                        
                    if sensor_type == "local":
                        server_sensor_data = {
                                "t": t,
                                "obj_id": obj_id,
                                "agent_name": obj_agent_name,
                                "velDist": obj_polar_vel[0],
                                "velAzim": obj_polar_vel[1],
                                "distance": obj_polar_coords[0],
                                "azimuth": obj_polar_coords[1],
                                "orientation": obj_orientation
                        }
                        
                    if sensor_type == "comm":
                        # в рамках сценария будем посылать по каналу коммуникации
                        # детекции дружеских агентов с камеры
                        
                        server_sensor_data = {
                            "t": t,
                            "obj_id": obj_id,
                            "agent_name": obj_agent_name,
                            "detections": []
                        }
                        for world_obj in all_world_obj:
                            world_obj_agent_name = world_obj["agent_name"]
                            if world_obj_agent_name == obj_agent_name:
                                continue
                            if not world_obj["friendly"] == obj_friendly:
                                continue
                            camera_last_iteration = server_data["camera"][curr_sensor_iteration - sensor_delta_time["camera"]]
                            if world_obj_agent_name in camera_last_iteration.keys():
                                server_sensor_data["detections"].append(camera_last_iteration[world_obj_agent_name]["detections"])
                                
                    
                    if obj_type == 2:                    
                        if sensor_type == "camera":
                            server_sensor_data = {
                                "t": t,
                                "obj_id": obj_id,
                                "agent_name": obj_agent_name,
                                "detections": []
                            }

                            long_zone_coords = get_triangle_angles(DRONE_CAMERA_LONG_ZONE, obj_evkl_coords, obj_orientation, DRONE_CAMERA_ANGEL_RAD)
                            middle_zone_coords = get_triangle_angles(DRONE_CAMERA_MIDDLE_ZONE, obj_evkl_coords, obj_orientation,DRONE_CAMERA_ANGEL_RAD)
                            close_zone_coords = get_triangle_angles(DRONE_CAMERA_CLOSE_ZONE, obj_evkl_coords, obj_orientation,DRONE_CAMERA_ANGEL_RAD)

                            for world_obj in all_world_obj:
                                if world_obj["agent_name"] == obj_agent_name:
                                    continue

                                world_obj_id = world_obj["obj_id"]
                                world_obj_agent_name = world_obj["agent_name"]
                                world_obj_evkl_vel = world_obj["velocity_evkl"]
                                world_obj_polar_vel = world_obj["velocity_polar"]
                                world_obj_evkl_coords = world_obj["coords_evkl"]
                                world_obj_polar_coords = world_obj["coords_polar"]
                                world_obj_type = world_obj["type"]
                                world_obj_class = world_obj["class"]
                                world_obj_friendly = world_obj["friendly"]

                                long_flag = obj_in_triangle(long_zone_coords[0],long_zone_coords[1], obj_evkl_coords, world_obj_evkl_coords)
                                middle_flag = obj_in_triangle(middle_zone_coords[0],middle_zone_coords[1], obj_evkl_coords, world_obj_evkl_coords)
                                close_flag = obj_in_triangle(close_zone_coords[0],close_zone_coords[1], obj_evkl_coords, world_obj_evkl_coords)

                                coords_noise = [0,0]
                                #проверяем принадлежность объекта к зоне
                                if close_flag:
                                    coords_noise[0] = np.random.normal(DRONE_CAMERA_DIST_CLOSE_ZONE_MEAN, DRONE_CAMERA_DIST_CLOSE_ZONE_DISP)
                                    coords_noise[1] = np.pi * np.random.normal(CLOSE_ANGLE_NOISE_MEAN, CLOSE_ANGLE_NOISE_DISP) / 180
                                    quality_type = np.random.randint(TYPE_CONF_CLOSE_ZONE[0], TYPE_CONF_CLOSE_ZONE[1])/100
                                    quality_class = np.random.randint(CLASS_CONF_CLOSE_ZONE, int(quality_type*100))/100
                                elif middle_flag:
                                    coords_noise[0] = np.random.normal(DRONE_CAMERA_DIST_MIDDLE_ZONE_MEAN, DRONE_CAMERA_DIST_MIDDLE_ZONE_DISP)
                                    coords_noise[1] = np.pi * np.random.normal(MIDDLE_ANGLE_NOISE_MEAN, MIDDLE_ANGLE_NOISE_DISP) / 180
                                    quality_type = np.random.randint(TYPE_CONF_MIDDLE_ZONE[0], TYPE_CONF_MIDDLE_ZONE[1])/100
                                    quality_class = np.random.randint(CLASS_CONF_MIDDLE_ZONE, int(quality_type*100))/100
                                elif long_flag:
                                    coords_noise[0] = np.random.normal(DRONE_CAMERA_DIST_LONG_ZONE_MEAN, DRONE_CAMERA_DIST_LONG_ZONE_DISP)
                                    coords_noise[1] = np.pi * np.random.normal(LONG_ANGLE_NOISE_MEAN, LONG_ANGLE_NOISE_DISP) / 180
                                    quality_type = np.random.randint(TYPE_CONF_LONG_ZONE[0], TYPE_CONF_LONG_ZONE[1])/100
                                    quality_class = -1.0
                                    world_obj_class = -1
                                else:
                                    continue

                                #смена знака координате расстояния на случай, если станет отрицательной
                                new_d = world_obj_polar_coords[0] + coords_noise[0]
                                if new_d < 0: new_d = 0.0
                                detection = {
                                        "category": 1,
                                        "obj_id": str(world_obj_id),
                                        "class": world_obj_class,
                                        "type": world_obj_type,
                                        "classConf": quality_class,
                                        "typeConf": quality_type,
                                        "distance":  new_d,
                                        "azimuth":  world_obj_polar_coords[1] + coords_noise[1],
                                        "friendly": world_obj_friendly
                                    }
                                server_sensor_data["detections"].append(detection)
                    elif sensor_type == "camera":
                        server_sensor_data = {
                                "t": t,
                                "obj_id": obj_id,
                                "agent_name": obj_agent_name,
                                "detections": []
                            }

                        long_zone_coords = get_triangle_angles(CAMERA_LONG_ZONE, obj_evkl_coords, obj_orientation, CAMERA_ANGEL_RAD)
                        middle_zone_coords = get_triangle_angles(CAMERA_MIDDLE_ZONE, obj_evkl_coords, obj_orientation, CAMERA_ANGEL_RAD)
                        close_zone_coords = get_triangle_angles(CAMERA_CLOSE_ZONE, obj_evkl_coords, obj_orientation, CAMERA_ANGEL_RAD)

                        for world_obj in all_world_obj:
                            if world_obj["agent_name"] == obj_agent_name:
                                continue

                            world_obj_id = world_obj["obj_id"]
                            world_obj_agent_name = world_obj["agent_name"]
                            world_obj_evkl_vel = world_obj["velocity_evkl"]
                            world_obj_polar_vel = world_obj["velocity_polar"]
                            world_obj_evkl_coords = world_obj["coords_evkl"]
                            world_obj_polar_coords = world_obj["coords_polar"]
                            world_obj_type = world_obj["type"]
                            world_obj_class = world_obj["class"]
                            world_obj_friendly = world_obj["friendly"]

                            long_flag = obj_in_triangle(long_zone_coords[0],long_zone_coords[1], obj_evkl_coords, world_obj_evkl_coords)
                            middle_flag = obj_in_triangle(middle_zone_coords[0],middle_zone_coords[1], obj_evkl_coords, world_obj_evkl_coords)
                            close_flag = obj_in_triangle(close_zone_coords[0],close_zone_coords[1], obj_evkl_coords, world_obj_evkl_coords)

                            
                            coords_noise = [0,0]
                            #проверяем принадлежность объекта к зоне
                            if close_flag:
                                coords_noise[0] = np.random.normal(CAMERA_DIST_CLOSE_ZONE_MEAN, CAMERA_DIST_CLOSE_ZONE_DISP)
                                coords_noise[1] = np.pi * np.random.normal(CLOSE_ANGLE_NOISE_MEAN, CLOSE_ANGLE_NOISE_DISP) / 180
                                quality_type = np.random.randint(TYPE_CONF_CLOSE_ZONE[0], TYPE_CONF_CLOSE_ZONE[1])/100
                                quality_class = np.random.randint(CLASS_CONF_CLOSE_ZONE, int(quality_type*100))/100
                            elif middle_flag:
                                coords_noise[0] = np.random.normal(CAMERA_DIST_MIDDLE_ZONE_MEAN, CAMERA_DIST_MIDDLE_ZONE_DISP)
                                coords_noise[1] = np.pi * np.random.normal(MIDDLE_ANGLE_NOISE_MEAN, MIDDLE_ANGLE_NOISE_DISP) / 180
                                quality_type = np.random.randint(TYPE_CONF_MIDDLE_ZONE[0], TYPE_CONF_MIDDLE_ZONE[1])/100
                                quality_class = np.random.randint(CLASS_CONF_MIDDLE_ZONE, int(quality_type*100))/100
                            elif long_flag:
                                coords_noise[0] = np.random.normal(CAMERA_DIST_LONG_ZONE_MEAN, CAMERA_DIST_LONG_ZONE_DISP)
                                coords_noise[1] = np.pi * np.random.normal(LONG_ANGLE_NOISE_MEAN, LONG_ANGLE_NOISE_DISP) / 180
                                quality_type = np.random.randint(TYPE_CONF_LONG_ZONE[0], TYPE_CONF_LONG_ZONE[1])/100
                                quality_class = -1.0
                                world_obj_class = -1
                            else:
                                continue

                            #смена знака координате расстояния на случай, если станет отрицательной
                            new_d = world_obj_polar_coords[0] + coords_noise[0]
                            if new_d < 0: new_d = 0.0
                            detection = {
                                    "category": 1,
                                    "obj_id": str(world_obj_id),
                                    "class": world_obj_class,
                                    "type": world_obj_type,
                                    "classConf": quality_class,
                                    "typeConf": quality_type,
                                    "distance":  new_d,
                                    "azimuth":  world_obj_polar_coords[1] + coords_noise[1],
                                    "friendly": world_obj_friendly
                                }
                            server_sensor_data["detections"].append(detection)
                    
                    #сохраняем сенсорные посылки и обновляет такт сенсора, 
                    #когда он должен сработать в следующий раз
                    if not server_sensor_data is None:
                        curr_sensor_iteration = sensor_counter[sensor_type]
                        if not curr_sensor_iteration in server_data[sensor_type].keys():
                            server_data[sensor_type][curr_sensor_iteration] = {}
                        
                        server_data[sensor_type][curr_sensor_iteration][obj_agent_name] = server_sensor_data
                    if len(stack_to_model) == 0:
                        sensor_timers[sensor_type] += sensor_delta_time[sensor_type]
                        sensor_counter[sensor_type] += 1
                    
    with open("server_data.json", "w") as f:
        f.write(json.dumps(server_data))
