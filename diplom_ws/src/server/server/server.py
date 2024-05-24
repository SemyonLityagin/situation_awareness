import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os, json

from sit_awar_msgs.msg import SensorPack
from sit_awar_msgs.msg import SensorData
from sit_awar_msgs.msg import SelfObjectState

sensors_time_period = {
    "radar": 0.2,
    "camera": 0.1,
    "local": 0.2,
    "comm": 4,
}
AGENT_CONFIG = "agent_config.json"
AGENT_CONFIG_DIR = "agent_config"
SCENARIO = "server_data.json"
SCENARIO_DIR = "scenario"

class Server(Node):
    def __init__(self):
        super().__init__('server')

        self.agent_publishers = {}
        self.sensor_timers = {}
        self.sensor_counter = {}

        # load agent_config
        agent_config_path = os.path.join(
            get_package_share_directory("sit_awar"),
            AGENT_CONFIG_DIR
            )
        self.get_logger().info(self.get_logger().name)
        with open(os.path.join(agent_config_path,AGENT_CONFIG), "r") as f:
            agent_config = json.loads(f.read())
        self.agent_name = agent_config["agent_name"]

        scenario_path = os.path.join(
            get_package_share_directory("server"),
            SCENARIO_DIR
            )
        with open(os.path.join(scenario_path,SCENARIO), "r") as f:
            self.scenario = json.loads(f.read())
    
        for sensor in agent_config["sensors"]:
            sensor_name = sensor["sensor"]
            sensor_topic = sensor["topic"]
            timer_period = sensors_time_period[sensor_name]
                
            # put zero in sensor counter to find sensorpack in serverdata.json from generator
            self.sensor_counter[sensor_name] = 0

            # set publisher for agent sensors
            if sensor_name == "local":
                self.agent_publishers[sensor_name] = self.create_publisher(SelfObjectState, "/"+self.agent_name+"/"+sensor_topic, 1)
            else:
                self.agent_publishers[sensor_name] = self.create_publisher(SensorPack, "/"+self.agent_name+"/"+sensor_topic, 1)
            
            # set timers for agent sensors
            if sensor_name == "local":        
                self.sensor_timers[sensor_name] = self.create_timer(timer_period, self.timer_local_callback)
            elif sensor_name == "radar":
                self.sensor_timers[sensor_name] = self.create_timer(timer_period, self.timer_radar_callback)
            elif sensor_name == "camera":
                self.sensor_timers[sensor_name] = self.create_timer(timer_period, self.timer_camera_callback)
            elif sensor_name == "comm":
                self.sensor_timers[sensor_name] = self.create_timer(timer_period, self.timer_comm_callback)

    def get_sensor_packs_comm_by_t_counter(self, sensor_name, t):
        comms = []
        try:
            comms = self.scenario[sensor_name][str(t)][self.agent_name]["detections"]
        except Exception as e:
            self.get_logger().info("{} {} {}".format(sensor_name, str(t), str(e)))
        
        msgs = []
        for detections in comms:
            msg = SensorPack()
            msg.sensor = sensor_name
            msg.objects = []

            for obj_detection in detections:
                obj = SensorData()
                obj.scen_id = str(obj_detection["obj_id"])
                obj.category = obj_detection["category"]
                obj.obj_class = obj_detection["class"]
                obj.obj_type = obj_detection["type"]
                obj.class_conf = obj_detection["classConf"]
                obj.type_conf = obj_detection["typeConf"]
                obj.distance = obj_detection["distance"]
                obj.azimuth = obj_detection["azimuth"]
                obj.friendly = obj_detection["friendly"]
                msg.objects.append(obj)
            msgs.append(msg)
        return msgs

    def get_sensor_pack_by_t_counter(self, sensor_name, t):
        detections = []
        try:
            detections = self.scenario[sensor_name][str(t)][self.agent_name]["detections"]
        except Exception as e:
            self.get_logger().info("{} {} {}".format(sensor_name, str(t), str(e)))
            
        msg = SensorPack()
        msg.sensor = sensor_name
        msg.objects = []

        for obj_detection in detections:
            obj = SensorData()
            obj.scen_id = str(obj_detection["obj_id"])
            obj.category = obj_detection["category"]
            obj.obj_class = obj_detection["class"]
            obj.obj_type = obj_detection["type"]
            obj.class_conf = obj_detection["classConf"]
            obj.type_conf = obj_detection["typeConf"]
            obj.distance = obj_detection["distance"]
            obj.azimuth = obj_detection["azimuth"]
            obj.friendly = obj_detection["friendly"]
            msg.objects.append(obj)
        return msg
    
    def get_self_state_by_t_counter(self, sensor_name, t):
        self_state = self.scenario[sensor_name][str(t)][self.agent_name]

        msg = SelfObjectState()

        msg.vel_dist = self_state["velDist"]
        msg.vel_azim = self_state["velAzim"]
        msg.distance = self_state["distance"]
        msg.azimuth = self_state["azimuth"]
        msg.orientation = self_state["orientation"]
        return msg

    def timer_radar_callback(self):
        sensor_name = "radar"
        t = self.sensor_counter[sensor_name]
        msg = self.get_sensor_pack_by_t_counter(sensor_name, t)
        self.agent_publishers[sensor_name].publish(msg)
        self.sensor_counter[sensor_name]+=1

    def timer_camera_callback(self):
        sensor_name = "camera"
        t = self.sensor_counter[sensor_name]
        msg = self.get_sensor_pack_by_t_counter(sensor_name, t)
        self.agent_publishers[sensor_name].publish(msg)
        self.sensor_counter[sensor_name]+=1

    def timer_comm_callback(self):
        sensor_name = "comm"
        t = self.sensor_counter[sensor_name]
        msgs = self.get_sensor_packs_comm_by_t_counter(sensor_name, t)
        for msg in msgs:
            self.agent_publishers[sensor_name].publish(msg)
        self.sensor_counter[sensor_name]+=1

    def timer_local_callback(self):
        sensor_name = "local"
        t = self.sensor_counter[sensor_name]
        msg = self.get_self_state_by_t_counter(sensor_name, t)
        self.agent_publishers[sensor_name].publish(msg)
        self.sensor_counter[sensor_name]+=1

def main(args=None):
    rclpy.init(args=args)

    server = Server()

    rclpy.spin(server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()