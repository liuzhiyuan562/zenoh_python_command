
import os
import time

import zenoh
from lanelet2.core import BasicPoint3d, GPSPoint
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector
from zenoh_ros_type.autoware_adapi_msgs import (
    ChangeOperationModeResponse,
    ClearRouteResponse,
    Route,
    RouteOption,
    SetRoutePointsRequest,
    SetRoutePointsResponse,
    VehicleKinematics,
)
from zenoh_ros_type.common_interfaces import (
    Header,
    Point,
    Pose,
    Quaternion,
)
from zenoh_ros_type.rcl_interfaces import Time
from zenoh_ros_type.tier4_autoware_msgs import GateMode

from map_parser import OrientationParser
from typing import cast
from pycdr2.types import sequence



GET_POSE_KEY_EXPR = '/api/vehicle/kinematics'
GET_GOAL_POSE_KEY_EXPR = '/api/routing/route'
SET_AUTO_MODE_KEY_EXPR = '/api/operation_mode/change_to_autonomous'
SET_ROUTE_POINT_KEY_EXPR = '/api/routing/set_route_points'
SET_CLEAR_ROUTE_KEY_EXPR = '/api/routing/clear_route'

### TODO: Should be replaced by ADAPI
SET_GATE_MODE_KEY_EXPR = '/control/gate_mode_cmd'


class VehiclePose:
    def __init__(self, session, scope):
        self.session = session
        self.scope = scope # vehicle name
        self.orientationGen = OrientationParser()
        self.publisher_gate_mode = self.session.declare_publisher(self.scope + SET_GATE_MODE_KEY_EXPR)

    def setGoal(self, coordinate_x, coordinate_y):

        # clear route
        clear_route = self.scope + SET_CLEAR_ROUTE_KEY_EXPR
        replies = self.session.get(clear_route)
        for reply in replies:
            try:
                print(">> Received ('{}':{})".format(reply.ok.key_expr, ClearRouteResponse.deserialize(reply.ok.payload.to_bytes())))
            except Exception as e:
                print(f'Failed to handle response (in {clear_route}): {e}')
        
        q = self.orientationGen.genQuaternion_seg(coordinate_x, coordinate_y)
        print('q: ', q)
        request = SetRoutePointsRequest(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id='map'),
            option=RouteOption(allow_goal_modification=False),
            goal=Pose(position=Point(x=coordinate_x, y=coordinate_y, z=0), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])),
            waypoints=cast(sequence[Pose], []),
        ).serialize()

        replies = self.session.get(self.scope + SET_ROUTE_POINT_KEY_EXPR, payload=request)
        for reply in replies:
            try:
                print(">> Received ('{}': {})".format(reply.ok.key_expr, SetRoutePointsResponse.deserialize(reply.ok.payload.to_bytes())))
            except Exception as e:
                print(f'Failed to handle response (in {SET_ROUTE_POINT_KEY_EXPR}): {e}')

    def engage(self):
        self.publisher_gate_mode.put(GateMode(data=GateMode.DATA['AUTO'].value).serialize())
        # Ensure Autoware receives the gate mode change before the operation mode change
        time.sleep(1)

        replies = self.session.get(self.scope + SET_AUTO_MODE_KEY_EXPR)
        for reply in replies:
            try:
                print(">> Received ('{}': {})".format(reply.ok.key_expr, ChangeOperationModeResponse.deserialize(reply.ok.payload.to_bytes())))
            except Exception as e:
                print(f'Failed to handle response (in {SET_AUTO_MODE_KEY_EXPR}): {e}')





class PoseServer:
    def __init__(self, session):
        self.session = session
        self.vehicles = {}
    
    def find_vehicles(self, times=10):
        for scope, vehicle in self.vehicles.items():
            vehicle.publisher_gate_mode.undeclare()

        self.vehicles = {}
        # for _ in range(times):
        while len(self.vehicles) == 0:
            replies = self.session.get('@/**/ros2/**' + GET_POSE_KEY_EXPR)
            for reply in replies:
                key_expr = str(reply.ok.key_expr)
                if 'pub' in key_expr:
                    end = key_expr.find(GET_POSE_KEY_EXPR)
                    vehicle = key_expr[:end].split('/')[-1]
                    print(f'find vehicle {vehicle}')
                    self.vehicles[vehicle] = None
            print('Waiting for vehicles to publish their pose...')
            time.sleep(3)
        self.construct_vehicle()
    def construct_vehicle(self):
        for scope in self.vehicles.keys():
            self.vehicles[scope] = VehiclePose(self.session, scope)


if __name__ == '__main__':
    conf = zenoh.Config.from_file('config.json5')
    session = zenoh.open(conf)
    pose_server = PoseServer(session)
    pose_server.find_vehicles()
    # pose_server.vehicles['v1'].setGoal(115.1446, -134.0757)
    # pose_server.vehicles['v1'].engage()
    pose_server.vehicles['hero'].setGoal(311.50691, -133.9085)
    pose_server.vehicles['hero'].engage()
    # time.sleep(1000)
