# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


from gc import callbacks
import numpy as np
import os
import datetime
from threading import Timer
from math import pi
import socket
import json
import time
import json, requests
import carb

from omni.isaac.examples.base_sample import BaseSample # Scenarios inherit from BaseSample
# Adding items to the scene
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
# General robot imports (simplified API from Isaac Sim to avoid Pixar's or other USD APIs)
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
# Franka Panda specific imports
from omni.isaac.franka import Franka
from omni.isaac.core.prims import RigidPrim

GRAD_TO_RAD = 2*pi/360
HOST_ADDR = "127.0.0.1"
PORT = 32323 # Port of the Azure Client App (or the simulation)

# From: https://stackoverflow.com/questions/3393612/run-certain-code-every-n-seconds
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        #self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

class PandaAIS(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # Here additional members can be defined such as _tasks, _controllers
        self._logger = None
        # TCP/IP socket to the Azure Client App
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # DEVELOP MEMBERS
        self._updateTimer = RepeatedTimer(0.2, self.update_joint_position_member)
        self._connectTimer = RepeatedTimer(2, self.connect_to_datasource)
        self._jointPos1 = 0.0
        self._turnDirectionPos = True
        return

    def setup_scene(self):
        # 1: Set up world scene 
        world = self.get_world()
        print("[INFO] Set up world scene.")
        #world.scene.add_default_ground_plane()

        # 2. Add groundplane to world scene 
        #world.scene.add(GroundPlane(prim_path="/World/groundPlane", size=12500, color=np.array([1.5, 1.5, 0.5])))
        print("[INFO] Added groundplane to scene.")
        # Use the find_nucleus_server instead of changing it every time
        # you configure a new server with /Isaac folder in it
        #assets_root_path = get_assets_root_path()
        #print("assets_root_path: ", assets_root_path)
        #if assets_root_path is None:
        #    # Use carb to log warnings, errors and infos in your application (shown on terminal)
        #    carb.log_error("Could not find nucleus server with /Isaac folder")
        #asset_path = assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        
        # 3. Add panda to world scene  
        absolute_asset_path = "/home/masais/panda/Franka/myjoghurt_ziyu.usd"
        # This will create a new XFormPrim and point it to the usd file as a reference
        # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=absolute_asset_path, prim_path="/World/myJoghurt")
        print("[INFO] Added plant to scene.")
        # Wrap the jetbot prim root under a Robot class and add it to the Scene
        # to use high level api to set/ get attributes as well as initializing
        # physics handles needed..etc.
        # Note: this call doesn't create the Jetbot in the stage window, it was already
        # created with the add_reference_to_stage
        
        # 4. Add casing to world scene
        # casing_prim_path = "/World/casing_basement"
        # #casing_absolute_asset_path = "/home/panda/Mir_casting_basement.usd"
        # casing_absolute_asset_path = "/home/panda/Mir_cart_casting.usd"
        # add_reference_to_stage(usd_path=casing_absolute_asset_path, prim_path=casing_prim_path)
        # # for basement demo
        # # casing = world.scene.add(RigidPrim(prim_path=casing_prim_path,
        # #                                    name="casing",
        # #                                    position=[0.37, -0.3, 0.1],
        # #                                    scale=[0.017]))
        # # only for automatica set up
        # casing = world.scene.add(RigidPrim(prim_path=casing_prim_path,
        #                                    name="casing",
        #                                    position=[0.56, -0.42, 0.1],
        #                                    scale=[0.008]))
        # print("[INFO] Added casing to scene.")

       



        ########



        # Add a cube for collision 
        # cube_prim_path = "/World/secu_cube"
        # secu_cube = world.scene.add(RigidPrim(prim_path=cube_prim_path, 
        #                                           name="secu_cube", 
        #                                           position=[0.4, 0.3, 0.6], 
        #                                           scale=[0.1, 0.2, 0.1], 
        #                                           color=np.array([1.0, 0.99, 0.0])))


        panda_robot = world.scene.add(Robot(prim_path="/World/myJoghurt/myJoghurt_ZIyu/franka", name="panda_robot",
                                            scale=[100]))
        
        
        

        print("Added robot to the scene")
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        print("Num of degrees of freedom before first reset: " + str(panda_robot.num_dof)) # prints None
        
        bottle =  panda_robot = world.scene.add(RigidPrim(prim_path="/World/myJoghurt/myJoghurt_ZIyu/Geometry/VN3066_000_Layout_Uni_Kassel_070919_0/VN3066_020_Band_Layout_1_91/_691296_GLAS_mit_Barcode_4_1591", 
                                                          name="bottle",
                                                          position=[-30,-50,10],
                                                        scale=[1]))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("panda_robot")
        self._bottle = self._world.scene.get_object("bottle")
        
        # self._casing = self._world.scene.get_object("casing") 
        # self._ring = self._world.scene.get_object("ring")
        # self._table = self._world.scene.get_object("table")

        print("------ Panda Infos ------")
        print("  DOF-Names     : ", self._franka.dof_names)
        print("  DOF-Properties: ", self._franka.dof_properties)
        current_bottle_pose = self._bottle.get_world_pose()
        print("Current bottle position: ", current_bottle_pose)


        print("---- Set-Up Logger ----")
        filepath = os.path.join("/home/masais/panda", "test_node2_full_integration.txt")
        self._logger = open(filepath, 'a')
        self._logger.write("\nStarting new entry at: {t}".format(t=datetime.datetime.now()))
        print("Finished logger setup")

        print("[SETUP] Setup the socket connection")
        self._connectTimer.start()        
        
        # This is an implicit PD controller of the jetbot/ articulation
        # setting PD gains, applying actions, switching control modes..etc.
        # can be done through this controller.
        # Note: should be only called after the first reset happens to the world
        
        #self._jetbot_articulation_controller = self._jetbot.get_articulation_controller()
        
        # Adding a physics callback to send the actions to apply actions with every
        # physics step executed.
        
        #self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        #self._world.add_physics_callback("logging_jointpos", callback_fn=self.log_joint_position_during_sim)
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # self._world.add_physics_callback("sending_actions", callback_fn=self.send_mir_actions)
        return
    
    def log_joint_position_during_sim(self, step_size):
        self._logger.write("\n{t} \nJoint-Pos.: {pos}\nJoint-Vel.: {vel}\nSoll: {d}".format(t=datetime.datetime.now(), 
                                                                                 pos=self._franka.get_joint_positions(), 
                                                                                 vel=self._franka.get_joint_velocities(),
                                                                                 d=self._jointPos1))
        return
    # def send_mir_actions(self,step_size):

    #     ip_mir = "192.168.50.250" # without ":8080"
    #     authorization_mir = "Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
    #     language_mir = "en_US"

    #     host = "http://" + ip_mir + "/api/v2.0.0/"
    #     headers = {}
    #     headers["Content-Type"] = "application/json"
    #     headers["Authorization"] = authorization_mir
    #     headers["Accept-Language"]= language_mir

    #     get_coord = requests.get(host + "/status", headers = headers).json()
    #     print(get_coord['position'])
    #     print(type(get_coord['position']['x']))
    #     position_mir = [get_coord['position']['y']-26,get_coord['position']['x']-25, 0]
    #     roll = get_coord['position']['orientation']/180*3.14
    #     pitch = 0
    #     yaw = 0
    #     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    #     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    #     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    #     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    #     orientation_mir = [qx, qy, qz, qw]

    #     self._casing.set_world_pose(position=position_mir, orientation=orientation_mir ) 
    #     return
    
    def send_robot_actions(self, step_size):
        # Every articulation controller has apply_action method
        # which takes in ArticulationAction with joint_positions, joint_efforts and joint_velocities
        # as optional args. It accepts numpy arrays of floats OR lists of floats and None
        # None means that nothing is applied to this dof index in this step
        # ALTERNATIVELY, same method is called from self._jetbot.apply_action(...)
        
        #self._jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
        #                                                                    joint_efforts=None,
        #                                                                    joint_velocities=5 * np.random.rand(2,)))
        
        # Test manually all moving partś and evaluate the necessary speed
        #if(not self._updateTimer.is_running):
        #    self._updateTimer.start()
        #current_pos = self._franka.get_joint_positions()
        #current_pos[0] = self._jointPos1    
        
        #self._franka.apply_action(control_actions = ArticulationAction(joint_positions=current_pos,
        #                                                               joint_efforts=None,
        #                                                               joint_velocities=None))

        #print("Step size: ", step_size)

        byte_buffer = self._client.recv(1000) # Read 1000 bytes from the tcp socket

        try:
            # First convert bytes to an utf-8 string
            string_buffer = byte_buffer.decode("utf-8") # utf-8 encoding seems to work fine
            print("Read data: ", string_buffer)
            # Get a list form this string (later convert to full JSON Object)
            json_list_buffer = json.loads(string_buffer)
            # Conver the json object to a np array
            nparray = np.array(json_list_buffer)
            print("nparray: ", nparray)

            if(nparray.size == 9):
                self._franka.set_joint_positions(nparray)
                # self._franka.apply_action(control_actions = ArticulationAction(joint_positions=nparray,
                #                                                         joint_efforts=None,
                #                                                         joint_velocities=None))
            elif(nparray.size == 7):
                print("Updated here: ", time.time())
                nparray = np.append(nparray, [0,0])
                #self._franka.apply_action(control_actions = ArticulationAction(joint_positions=nparray,
                #                                                        joint_efforts=None,
                #                                                        joint_velocities=None))
                self._logger.write("\n{t} Joint-Pos.: {pos}".format(t=datetime.datetime.now(), 
                                                                    pos=string_buffer))
                self._franka.set_joint_positions(nparray)
                # self._franka.apply_action(control_actions = ArticulationGripper([0]))
            elif(nparray.size >= 11):
                franka_positions = nparray[:9]
                bottle_position = nparray[9:11]

                self._franka.set_joint_positions(franka_positions)
                self._bottle.set_world_pose(position=[bottle_position[0], bottle_position[1],self._bottle.get_world_pose()[2]])


            else:
                print("Skipped array of incompatible size: ", nparray.size)
        
        except Exception as ex:
            print(f"Exception while converting data: {ex}")    

        return

    async def setup_pre_reset(self):
        #self._updateTimer.stop()
        #self._connectTimer.start()
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        #self._updateTimer.stop()
        return

    #------------------------------------------#
    # Unrelated to Omnvierse utility functions #
    #------------------------------------------#
    def update_joint_position_member(self):
        if(self._turnDirectionPos and self._jointPos1 <= 89.5*GRAD_TO_RAD):
            self._jointPos1 += 0.1
        elif(self._turnDirectionPos and self._jointPos1 > 89.5*GRAD_TO_RAD):
            self._turnDirectionPos = False
        elif(not self._turnDirectionPos and self._jointPos1 >= -89.5*GRAD_TO_RAD):
            self._jointPos1 -= 0.1
        else:
            self._turnDirectionPos = True
        
    def connect_to_datasource(self):
        print("Connecting to simulation on port: ", PORT)
        try:
            self._client.connect((HOST_ADDR, PORT))
            print("[DONE] Connected to the server app")
            self._connectTimer.stop()
        except:
            print("[ERROR] Could not connect to a server app")
