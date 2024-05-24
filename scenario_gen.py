import numpy as np
import cv2
import json
import math
from scen_to_server import get_triangle_angles, DRONE_CAMERA_ANGEL_RAD, CAMERA_ANGEL_RAD
from scen_to_server import DRONE_CAMERA_LONG_ZONE, CAMERA_LONG_ZONE

# type and class[type_key] dicts
type_data = {
        1: "Car",
        2: "BPLA"
    }
class_data = {
        1: {
            1: "Leopard",
            2: "M109",
            3: "OurPlatform"
        },
        2: {
            1: "Drone"
        }
    }

def getMachine(obj, motion_model, coords, plan=None):
    try:
        if class_data[obj["type"]][obj["class"]] == "Leopard":
            return Leopard(obj["obj_id"], motion_model, coords,plan)
        if class_data[obj["type"]][obj["class"]] == "M109":
            return M109(obj["obj_id"], motion_model, coords,plan)
        if class_data[obj["type"]][obj["class"]] == "Drone":
            return Drone(obj["obj_id"], motion_model, coords,plan)
        if class_data[obj["type"]][obj["class"]] == "OurPlatform":
            return OurPlatform(obj["obj_id"], motion_model, coords,plan)
    except Exception as e:    
        raise Exception("No such type or class of Machine\n" + str(e))

class EquidistantMotionModel:
    def __init__(self, acceleration, velocity):
        self.acceleration = acceleration
        self.velocity = velocity
        self.max_velocity = None
        self.pause = False

    def step(self, t):
        pass
        
    def get_coords(self, coords, t):
        pass

    def pause_motion(self):
        self.pause = True
    
    def set_max_velocity(self, max_velocity):
        self.max_velocity = max_velocity

    def update_acceleration(self, acceleration):
        self.acceleration = acceleration
        self.velocity = np.linalg.norm(self.velocity)*acceleration / np.linalg.norm(acceleration)

class EquidistantEvklidMotionModel(EquidistantMotionModel):
    def step(self, t):
        if not self.pause:
            self.velocity += self.acceleration*t
            if self.max_velocity and np.linalg.norm(self.velocity) > self.max_velocity:
                self.velocity = self.max_velocity * self.velocity / np.linalg.norm(self.velocity)
    
    def get_coords(self, coords, t):
        if self.pause:
            return coords
        return coords + self.velocity*t + (self.acceleration*t**2)/2
    

class Machine:
    def __init__(self, obj_id, motion_model, coords, plan = None):
        self.obj_id = obj_id
        self.motion_model = motion_model
        self.coords = coords
        self.previous_polar_coords = np.zeros((2))
        self.plan = plan
        self.plan_goal_idx = 0
        self.acceleration_scale = 2
        self.previous_dist_to_goal = 0

    def get_theta(self, x, y):
        if x == 0 and y == 0:
            return 0
    
    def get_polar_coords(self):
        r = np.linalg.norm(self.coords)
        theta = np.arctan2(self.coords[1],self.coords[0])
        return np.array([r, theta])
    
    def get_previous_polar_coords(self):
        return self.previous_polar_coords

    def get_evkl_coords(self):
        return self.coords

    def get_evkl_vel(self):
        return self.motion_model.velocity

    def check_goal(self):
        if self.plan_goal_idx >= len(self.plan):
            self.motion_model.pause_motion()
            self.plan = None
        else:
            new_dist_to_goal = np.sqrt(((self.coords - self.plan[self.plan_goal_idx])**2).sum())
            if new_dist_to_goal > self.previous_dist_to_goal:
                new_goal_vector = self.plan[self.plan_goal_idx] - self.coords
                self.motion_model.update_acceleration(self.acceleration_scale * new_goal_vector/np.linalg.norm(new_goal_vector))
                
            if new_dist_to_goal < 10:
                self.plan_goal_idx += 1
                if self.plan_goal_idx >= len(self.plan):
                    print("Agent{} finished!".format(self.obj_id))
                    self.motion_model.pause_motion()
                    self.plan = None
                else:
                    print("Agent{} update goal to {}".format(self.obj_id, self.plan[self.plan_goal_idx]))
                    new_goal_vector = self.plan[self.plan_goal_idx] - self.coords
                    self.motion_model.update_acceleration(self.acceleration_scale * new_goal_vector/np.linalg.norm(new_goal_vector))

    def step(self, t):
        if not self.plan is None:
            self.check_goal()
            if not self.plan is None:
                self.previous_dist_to_goal = np.sqrt(((self.coords - self.plan[self.plan_goal_idx])**2).sum())

        self.previous_polar_coords = self.get_polar_coords()
        self.motion_model.step(t)
        self.coords = self.motion_model.get_coords(self.coords, t)
        
class Leopard(Machine):
    def __init__(self, obj_id, motion_model, coords, plan = None):
        super().__init__(obj_id, motion_model, coords, plan)
        self.obj_class = 1
        self.obj_type = 1
        self.width = 3.7
        self.length = 7.7
        self.max_velocity = 40
        self.motion_model.set_max_velocity = self.max_velocity
        self.acceleration_scale = 2

class M109(Machine):
    def __init__(self, obj_id, motion_model, coords, plan = None):
        super().__init__(obj_id, motion_model, coords, plan)
        self.obj_class = 2
        self.obj_type = 1
        self.width = 3.2
        self.length = 6.1
        self.max_velocity = 30
        self.motion_model.set_max_velocity = self.max_velocity
        self.acceleration_scale = 2

class Drone(Machine):
    def __init__(self, obj_id, motion_model, coords, plan = None):
        super().__init__(obj_id, motion_model, coords, plan)
        self.obj_class = 1
        self.obj_type = 2
        self.width = 0.5
        self.length = 0.5
        self.max_velocity = 200
        self.motion_model.set_max_velocity = self.max_velocity
        self.acceleration_scale = 5

class OurPlatform(Machine):
    def __init__(self, obj_id, motion_model, coords, plan = None):
        super().__init__(obj_id, motion_model, coords, plan)
        self.obj_class = 3
        self.obj_type = 1
        self.width = 6
        self.length = 7
        self.max_velocity = 200
        self.motion_model.set_max_velocity = self.max_velocity
        self.acceleration_scale = 2

# FOR OLD VERSION/DON'T WORK NOW
# def generate_models_randomly(enemy, ally, map_width, map_length):
#     enemy_list = []
#     ally_list = []
#     min_map_size = min(map_width, map_length)
#     coord_mean = min_map_size/2
#     coord_std = min_map_size/8
    
#     for idx,en in enumerate(enemy):
#         coords = np.random.normal(coord_mean, coord_std, 2)
#         velocity = np.zeros(2)
#         acceleration = np.random.normal(1, 0.5, 2) if en["move"] else np.zeros(2)
#         motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
#         enemy_list.append(getMachine(en, motion_model, coords))

#     for idx,al in enumerate(ally):
#         coords = np.random.normal(min_map_size*3/4, coord_std, 2)
#         velocity = np.zeros(2)
#         acceleration = np.random.normal(1, 0.5, 2) if al["move"] else np.zeros(2)
#         motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
#         ally_list.append(getMachine(en, motion_model, coords))
#     return enemy_list, ally_list

# FOR OLD VERSION/DON'T WORK NOW
# def generate_models_uniformly(enemy, ally, map_width, map_length):
#     enemy_list = []
#     ally_list = []
#     min_map_size = min(map_width, map_length)
#     coord_step = min_map_size/16

#     free_coords = []
#     for x in range(1, int(min_map_size/coord_step)):
#         for y in range(1, int(min_map_size/coord_step)):
#             free_coords.append([x*coord_step,y*coord_step])
#     free_coords = np.array(free_coords)
    
#     for idx,en in enumerate(enemy):
#         coords = free_coords[idx+1]
#         velocity = np.zeros(2)
#         acceleration = np.random.normal(1, 0.5, 2) if en["move"] else np.zeros(2)
#         motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
#         if class_data[en["type"]][en["class"]] == "Leopard":
#             enemy_list.append(Leopard(en["obj_id"], motion_model, coords))
#         if class_data[en["type"]][en["class"]] == "M109":
#             enemy_list.append(M109(en["obj_id"], motion_model, coords))

#     for idx,al in enumerate(ally):
#         coords = free_coords[-idx-1]
#         velocity = np.zeros(2)
#         acceleration = np.random.normal(1, 0.5, 2) if al["move"] else np.zeros(2)
#         motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
#         if class_data[al["type"]][al["class"]] == "Drone":
#             ally_list.append(Drone(al["obj_id"], motion_model, coords))
#         if class_data[al["type"]][al["class"]] == "OurPlatform":
#             ally_list.append(OurPlatform(al["obj_id"], motion_model, coords))
#     return enemy_list, ally_list

def generate_models_circly(enemy, ally, map_width, map_length):
    enemy_list = []
    ally_list = []
    min_map_size = min(map_width, map_length)
    coord_step = min_map_size/16

    move_order = [False, True]
    enemy.sort(key=lambda x: move_order.index(x['move']))
    ally.sort(key=lambda x: move_order.index(x['move']))

    base_enemy_coord = [coord_step,coord_step]
    base_enemy_acceleration = [0,1.1]
    free_coords = [base_enemy_coord]
    free_acceleration = [base_enemy_acceleration]
    number_enemy = len(enemy)
    ring_koef = 1
    i_dropper = 0
    for i in range(number_enemy-1):
        alpha = np.pi*(i-i_dropper)/(3)/ring_koef
        if alpha > np.pi/2:
            i_dropper = i+1
            ring_koef+=1
            alpha = np.pi*(i-i_dropper)/(3)/ring_koef
        koef_x = np.sin(alpha)
        koef_y = np.cos(alpha)
        free_coords.append([base_enemy_coord[0] + coord_step*ring_koef*koef_x, base_enemy_coord[1] + coord_step*ring_koef*koef_y])
        free_acceleration.append([base_enemy_acceleration[0]+koef_x, base_enemy_acceleration[1]+koef_y])
    free_coords = np.array(free_coords)
    free_acceleration = np.array(free_acceleration)
    
    for idx,en in enumerate(enemy):
        coords = free_coords[idx]
        velocity = np.zeros(2)
        acceleration = free_acceleration[idx] if en["move"] else np.zeros(2)
        motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
        enemy_list.append(getMachine(en, motion_model, coords))

    base_ally_coord = [map_width-coord_step,map_length-coord_step]
    base_ally_acceleration = [0,-1.1]
    free_coords = [base_ally_coord]
    free_acceleration = [base_ally_acceleration]
    number_ally = len(ally)
    ring_koef = 1
    i_dropper = 0
    coord_step = 1000
    for i in range(number_ally-1):
        alpha = np.pi*(i-i_dropper)/(3)/ring_koef-np.pi
        if alpha > -np.pi/2:
            i_dropper = i+1
            ring_koef+=1
            alpha = np.pi*(i-i_dropper)/(3)/ring_koef-np.pi
        koef_x = np.sin(alpha)
        koef_y = np.cos(alpha)
        free_coords.append([base_ally_coord[0] + coord_step*ring_koef*koef_x, base_ally_coord[1] + coord_step*ring_koef*koef_y])
        free_acceleration.append([base_ally_acceleration[0]+koef_x, base_ally_acceleration[1]+koef_y])
    free_coords = np.array(free_coords)
    free_acceleration = np.array(free_acceleration)
    
    for idx,al in enumerate(ally):
        coords = free_coords[idx]
        velocity = np.zeros(2)
        acceleration = free_acceleration[idx] if al["move"] else np.zeros(2)
        motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
        ally_list.append(getMachine(al, motion_model, coords))
    return enemy_list, ally_list


def generate_models_plan(enemy, ally, map_width=None, map_length=None):
    enemy_list = []
    ally_list = []

    move_order = [False, True]
    enemy.sort(key=lambda x: move_order.index(x['move']))
    ally.sort(key=lambda x: move_order.index(x['move']))

    for idx,en in enumerate(enemy):
        plan = np.array(en["plan"])
        coords = plan[0]
        velocity = np.zeros(2)
        acceleration = np.zeros(2)
        motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
        enemy_list.append(getMachine(en, motion_model, coords, plan))
    
    for idx,al in enumerate(ally):
        plan = np.array(al["plan"])
        coords = plan[0]
        velocity = np.zeros(2)
        acceleration = np.zeros(2)
        motion_model = EquidistantEvklidMotionModel(acceleration, velocity)
        ally_list.append(getMachine(al, motion_model, coords, plan))
    return enemy_list, ally_list

def get_objects_data_for_scenario(object_list, dt, friendly):
    object_t = []
    for obj in object_list:
        polar_coords = obj.get_polar_coords()
        previous_polar_coords = obj.get_previous_polar_coords()
        evkl_coords = obj.get_evkl_coords()
        evkl_vel = obj.get_evkl_vel()
        object_t.append({
            "obj_id": obj.obj_id,
            "agent_name": "agent"+str(obj.obj_id),
            "class": obj.obj_class,
            "type": obj.obj_type,
            "width": obj.width,
            "length": obj.length,
            "coords_polar": polar_coords.tolist(),
            "coords_evkl": evkl_coords.tolist(),
            "velocity_evkl": evkl_vel.tolist(),
            "velocity_polar": (polar_coords - previous_polar_coords).tolist(),
            "friendly": friendly
            })
        obj.step(dt)
    return object_t

def generate_scenario(scenario_filename, generate_type, enemy, ally, map_width, map_length, dt, t):
    json_scenario = {
            "map_width": map_width,
            "map_length": map_length,
            "t": t,
            "iterations": []
        }
    if generate_type == "circly":
        enemy_list, ally_list = generate_models_circly(enemy,ally,map_width,map_length)
    elif generate_type == "plan":
        enemy_list, ally_list = generate_models_plan(enemy,ally,map_width,map_length)
    # THIS GENERATOR TYPE DON'T WORK IN CURRENT VERSION
    # elif generate_type == "uniformly":
    #     enemy_list, ally_list = generate_models_uniformly(enemy,ally,map_width,map_length)
    # elif generate_type == "randomly":
    #     enemy_list, ally_list = generate_models_randomly(enemy,ally,map_width,map_length)
    else:
        raise Exception("Choose generate_type: randomly, circly or uniformaly")
    for iteration in range(t):
        enemy_t = get_objects_data_for_scenario(enemy_list, dt, False)
        ally_t = get_objects_data_for_scenario(ally_list, dt, True)
        
        json_scenario["iterations"].append({
            "t": dt*iteration,
            "enemy": enemy_t,
            "ally":ally_t
        })

    with open(scenario_filename, "w") as f:
        f.write(json.dumps(json_scenario))

def draw_objects(object_list, map_image, color, scale):
    object_number = len(object_list)

    for obj in object_list:
        x,y = obj["coords_evkl"]
        x = int(x/scale)
        y = int(y/scale)
        vx,vy = obj["velocity_evkl"]
        
        obj_orientation = None
        left = None
        right = None
        
        if not(vx == 0 and vy == 0):
            obj_orientation = np.sign(vy)*np.arccos(vx/(math.sqrt(vx**2 + vy**2)))
            
            obj_class = class_data[obj["type"]][obj["class"]]
            if obj_class == "Drone":
                left, right = get_triangle_angles(DRONE_CAMERA_LONG_ZONE/scale, [x,y], obj_orientation, DRONE_CAMERA_ANGEL_RAD)
            else:
                left, right = get_triangle_angles(CAMERA_LONG_ZONE/scale, [x,y], obj_orientation, CAMERA_ANGEL_RAD)
                
        r, theta = obj["coords_polar"]
        w = obj["width"]
        l = obj["length"]
        font_scale = 0.3
        thickness = 1
        if x < 0 or x > map_image.shape[1]:
            continue
        if y < 0 or y > map_image.shape[0]:
            continue
        
        if map_image.shape[0]<map_image.shape[1]:
            font_scale = (map_image.shape[0]/40)/(map_image.shape[1]/80)
            thickness = int(np.round(map_image.shape[0]/map_image.shape[1]))
        else:
            font_scale = (map_image.shape[1]/40)/(map_image.shape[0]/80)
            thickness = int(np.round(map_image.shape[1]/map_image.shape[0]))

        if map_image.shape[0] > 200 and map_image.shape[1] > 200 and object_number*25 < 0.5*map_image.shape[1]*map_image.shape[0]:

            if not obj_orientation is None:
                cv2.line(map_image, (x, y), (int(left[0]), int(left[1])), color, thickness=thickness)
                cv2.line(map_image, (x, y), (int(right[0]), int(right[1])), color, thickness=thickness)

            cv2.putText(map_image, class_data[obj["type"]][obj["class"]] + "_" + str(obj["obj_id"]),
            (x,y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255,255,255),
            thickness,
            2)

            obj_r = int((w+l)/4/scale)
        else:
            obj_r = 1
        cv2.circle(map_image, (x,y), obj_r, color, 1)


def visualize_scenario(scenario_filename,scale):
    with open(scenario_filename, "r") as f:
        json_scenario = json.loads(f.read())

    enemy_color = (0, 0, 255)
    ally_color = (0, 255, 0)
    map_length = int(json_scenario["map_length"]/scale)
    map_width = int(json_scenario["map_width"]/scale)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    output_file = scenario_filename.split(".")[0] + "_" + str(scale) + "_video.mp4"
    out = cv2.VideoWriter(output_file, fourcc, 10, (map_width, map_length))

    for iteration in json_scenario["iterations"]:
        map_image = np.zeros((map_length, map_width, 3), dtype=np.uint8)
        draw_objects(iteration["enemy"], map_image, enemy_color, scale)
        draw_objects(iteration["ally"], map_image, ally_color, scale)
        out.write(map_image)

    out.release()
    

if __name__=="__main__":
    scenario_filename = "scenario.json"
    generate_type = "plan"
    map_width = 2000
    map_length = 2000

    ally = [
        {
            "class": 3,
            "type": 1,
            "obj_id": 0,
            "move": False,
            "plan": [
                    [100, 100],
                    [101, 101]
                ]
        },
        {
            "class": 1,
            "type": 2,
            "obj_id": 1,
            "move": True,
            "plan": [
                    [150, 100],
                    [1500, 500],
                    [1300, 602]
                ]
        },
        {
            "class": 1,
            "type": 2,
            "obj_id": 2,
            "move": True,
            "plan": [
                    [100, 150],
                    [500, 1500],
                    [600, 1300]
                ]
        },
        {
            "class": 1,
            "type": 2,
            "obj_id": 10,
            "move": False,
            "plan": [
                    [200, 150],
                    [201, 150]
                ]
        }
    ]

    enemy = [
        {
            "class": 1,
            "type": 1,
            "obj_id": 3,
            "move": False,
            "plan": [
                    [1700, 1700],
                    [1699, 1699]
                ]
        },
        {
            "class": 2,
            "type": 1,
            "obj_id": 4,
            "move": True,
            "plan": [
                    [1600, 1700],
                    [600, 1000]
                ]
        },
        {
            "class": 2,
            "type": 1,
            "obj_id": 5,
            "move": True,
            "plan": [
                    [1700, 1600],
                    [1000, 600]
                ]
        },
    ]

    # WAS NEED FOR TEST SIT_AWAR ON COMPLEX SCENARIO
    # enemy = []
    # id = 1
    # test_classes = [1,2,3] #,3
    # classes = {}
    # for c in test_classes: classes[c] = 0
    # grid_size = 5
    # for i in range(0,10):
    #     for j in range(0,10):
    #         if i == 0 and j == 0: continue
    #         pose = [grid_size*i, grid_size*j]
    #         pose2 = [grid_size*i+500, grid_size*j+500]
    #         if np.sqrt((pose[0] - ally[0]["plan"][0][0])**2+(pose[1] - ally[0]["plan"][0][1])**2) <= 50:
    #             enemy.append({
    #                 "class": np.random.choice(test_classes),
    #                 "type": 1,
    #                 "obj_id": id,
    #                 "move": True,
    #                 "plan": [
    #                         pose, pose2
    #                     ]
    #             })
    #             classes[enemy[-1]["class"]]+=1
    #             id+=1

    dt = 0.1
    t = 500
    scale = 1
    
    generate_scenario(generate_type + "_" + scenario_filename, generate_type, enemy, ally, map_width, map_length, dt, t)
    visualize_scenario(generate_type + "_" +  scenario_filename, scale)
    
    
