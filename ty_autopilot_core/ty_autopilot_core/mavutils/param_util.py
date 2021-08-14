#!/usr/bin/env python3
from functools import cached_property
import rclpy
import rclpy.node 
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from mavros_msgs.srv import ParamGet, ParamSet, ParamPull
from mavros_msgs.msg import ParamValue
from rosidl_parser.definition import BOOLEAN_TYPE
from ty_autopilot_core.base_node import BaseNode
import typing
import yaml

SERVICE_WAIT_TIMEOUT = 5.0

class ServiceWaitTimeout(RuntimeError):
    pass

def wait_for_service(client: rclpy.node.Client,lg: typing.Optional[typing.Any] ):
    ready = client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT)
    if not ready:
        topic = client.srv_name
        if lg:
            lg.error("wait for service timeout: {topic}")
        raise ServiceWaitTimeout(topic)


class MavParam(BaseNode):
    def __init__(self):
        BaseNode.__init__(self)
               
    def mav_param_get(self,param_id):
        try:
            get = self.create_client(ParamGet, '/mavros/param/get')
            wait_for_service(get, self.get_logger())
            req = ParamGet.Request()
            req.param_id = param_id
            ret = get.call(req)
        except Exception as ex:
            self.get_logger().info('Service call failed %r' % (ex))
        if not ret.success:
            raise IOError("Request failed.")

        if ret.value.integer != 0:
            return ret.value.integer
        elif ret.value.real != 0.0:
            return ret.value.real
        else:
            return 0

    def mav_param_set(self,param_id, value):
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)
        set = self.create_client(ParamSet, '/mavros/param/set')
        # wait_for_service(set, self.get_logger())
        req = ParamSet.Request()
        req.param_id = param_id
        req.value = val
        ret = set.call(req)
        if not ret.success:
            raise IOError("Request failed.")
        
        if ret.value.integer != 0:
            return ret.value.integer
        elif ret.value.real != 0.0:
            return ret.value.real
        else:
            return 0

    def mav_param_pull(self,force):
        pull = self.create_client(ParamPull, '/mavros/param/pull')
        wait_for_service(pull, self.get_logger())
        req = ParamPull.Request()
        req.force_pull = True
        ret = pull.call(req)
        if not ret.success:
            raise IOError("Request failed.")

        return ret.success


class RosParam:
    # def __init__(self):
    #     BaseNode.__init__(self)

    def parse_yaml(self,path):
        with open(path, 'r') as file:
            configParams = yaml.safe_load(file) #[self.get_name()]['ros__parameters'] 
        return configParams

    def get_param(self,parameter_name, default_value = None, get_value=True):
        # if parameter_name.startswith("/"):
        #     BaseNode.get_logger(self).info("Getting parameters of other nodes is not yet supported")
        #     return 0

        # parameter_name = parameter_name.strip("~")
        if not BaseNode.has_parameter(self, parameter_name):
            BaseNode.declare_parameter(self, parameter_name, default_value)
        if get_value:
            return BaseNode.get_parameter(self,parameter_name)._value
        else:
            return BaseNode.get_parameter(self, parameter_name)


    def set_param(self, parameter_name, parameter_value):
        if type(parameter_value) is str:
            parameter_type = rclpy.Parameter.Type.STRING
        elif type(parameter_value) is float:
            parameter_type = rclpy.Parameter.Type.DOUBLE
        elif type(parameter_value) is int:
            parameter_type = rclpy.Parameter.Type.INTEGER
        elif type(parameter_value) is bool:
            parameter_type = rclpy.Parameter.Type.BOOL
        else:
            raise Exception("Invalid parameter value type: %s" % str(type(parameter_value)))

        param = Parameter(
            parameter_name,
            parameter_type,
            parameter_value,
        )
        print(param)
        BaseNode.set_parameters(self,[param])