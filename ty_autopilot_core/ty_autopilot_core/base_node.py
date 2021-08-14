from typing import Optional, Union
import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
import rclpy.qos
import threading
from rclpy.executors import Executor
from functools import cached_property
from dataclasses import dataclass
from functools import partial

from rclpy.qos_event import PublisherEventCallbacks

SERVICE_WAIT_TIMEOUT = 5

@dataclass
class AutoilotParams:
    test_param: int = 1
    

class BaseNode(rclpy.node.Node):
    _ns: str

    def __init__(self, node_name: str = 'ty_autopilot_core_node', ns: str = 'ty_autopilot_core'):
        super().__init__(node_name)
        self._ns = ns
        self.message = {}
        self.executor = Executor()
        self.status_qos = rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sensors_qos = rclpy.qos.qos_profile_sensor_data
        self.parameters_qos = rclpy.qos.qos_profile_parameters

    @property
    def ns(self) -> str:
        return self._ns

    def get_topic(self, *args: str) -> str:
        return '/'.join((self._ns, ) + args)

    def subscribe_topic(self, topic, type, qos_profile):
        self.message[topic] = type
        cb_function = partial(self.listen_topic, topic = topic)
        self.create_subscription(type, topic, cb_function, qos_profile)

    def listen_topic(self, msg, topic):
        self.message[topic] = msg

    def read_topic(self, topic):
        return self.message[topic]
    
    def Publisher(self, type, topic, qos_profile) -> Publisher:
        return self.create_publisher(type, topic, qos_profile)

    def add_to_executor(self, process):
        self.executor.add_node(process)
    
    def create_srv_client(self, srv_type, srv_name, qos_profile, callback):
        return self.create_client(srv_type, srv_name, qos_profile, callback)