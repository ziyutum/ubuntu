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
        world.scene.add(GroundPlane(prim_path="/World/groundPlane", size=25, color=np.array([0.5, 0.5, 0.5])))
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
        absolute_asset_path = "/home/panda/Franka/franka.usd"
        # This will create a new XFormPrim and point it to the usd file as a reference
        # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=absolute_asset_path, prim_path="/World/Panda_Robot")
        print("[INFO] Added panda to scene.")
        # Wrap the jetbot prim root under a Robot class and add it to the Scene
        # to use high level api to set/ get attributes as well as initializing
        # physics handles needed..etc.
        # Note: this call doesn't create the Jetbot in the stage window, it was already
        # created with the add_reference_to_stage
        
        # 4. Add casing to world scene
        casing_prim_path = "/World/casing_basement"
        #casing_absolute_asset_path = "/home/panda/Mir_casting_basement.usd"
        casing_absolute_asset_path = "/home/panda/Mir_cart_casting.usd"
        add_reference_to_stage(usd_path=casing_absolute_asset_path, prim_path=casing_prim_path)
        # for basement demo
        # casing = world.scene.add(RigidPrim(prim_path=casing_prim_path,
        #                                    name="casing",
        #                                    position=[0.37, -0.3, 0.1],
        #                                    scale=[0.017]))
        # only for automatica set up
        casing = world.scene.add(RigidPrim(prim_path=casing_prim_path,
                                           name="casing",
                                           position=[0.56, -0.42, 0.1],
                                           scale=[0.008]))
        print("[INFO] Added casing to scene.")

        # 5. Add the design-ring to the scene
        ring_prim_path = "/World/ring"
        ring_absolute_asset_path = "/home/panda/designring.usd"
        add_reference_to_stage(usd_path=ring_absolute_asset_path, prim_path=ring_prim_path)
        # position=[0.35, 0.3, 0.6] sollte sein
        ring = world.scene.add(RigidPrim(prim_path=ring_prim_path,
                                           name="ring",
                                           position=[0.5, 0.3, 0.6],
                                           scale=[0.017]))
        print("[INFO] Added design ring to scene.")
        
        # 6. Todos: Add other objects to scene
        ring_prim_path = "/World/table"
        ring_absolute_asset_path = "/home/panda/Downloads/CAD_to_USD/manipulation_module_cad_141511/manipulation_module_cad/total_assembly_tender_reducedsize.usd"
        add_reference_to_stage(usd_path=ring_absolute_asset_path, prim_path=ring_prim_path)
        ring = world.scene.add(RigidPrim(prim_path=ring_prim_path,
                                          name="table",
                                          position=[0, 1, 0],
                                          scale=[0.0065]))
        # change scale to 0.005 in automatica, the original one in DM-demo is 0.007
        
        # Station W9 
        # table
        t9_prim_path = "/World/table_w9"
        t9_absolute_asset_path = "/home/panda/total_assembly_tender_reducedsize_v2.usd"
        add_reference_to_stage(usd_path=t9_absolute_asset_path, prim_path=t9_prim_path)
        t9 = world.scene.add(RigidPrim(prim_path=t9_prim_path,
                                          name="table_w9",
                                          position=[3, 1.2, 0],
                                          #orientation = [1,0,0,-2],
                                          scale=[0.005]))
        # add_reference_to_stage(usd_path=absolute_asset_path, prim_path="/World/panda_dummy_w9")
        # panda_dummy_w9 = world.scene.add(Robot(prim_path="/World/panda_dummy_w9", name="panda_dummy_w9", 
        #                                     position = [3.3, 0.8, 0.43],
        #                                     orientation = [1,0,0,-1]))
        
        t9_2_prim_path = "/World/table_w9_2"
        t9_2_absolute_asset_path = "/home/panda/total_assembly_tender_reducedsize_v2.usd"
        add_reference_to_stage(usd_path=t9_2_absolute_asset_path, prim_path=t9_2_prim_path)
        t9_2 = world.scene.add(RigidPrim(prim_path=t9_2_prim_path,
                                          name="table_w9_2",
                                          position=[2.3, 1.2, 0],
                                          #orientation = [1,0,0,-2],
                                          scale=[0.005]))
        # add_reference_to_stage(usd_path=absolute_asset_path, prim_path="/World/panda_dummy_w9_2")
        # panda_dummy_w9_2 = world.scene.add(Robot(prim_path="/World/panda_dummy_w9_2", name="panda_dummy_w9_2", 
        #                                     position = [2.6, 0.8, 0.43],
        #                                     orientation = [1,0,0,-1]))


        # Station W10
        t10_prim_path = "/World/table_w10"
        t10_absolute_asset_path = "/home/panda/basic_table_automatica_v2.usd"
        add_reference_to_stage(usd_path=t10_absolute_asset_path, prim_path=t10_prim_path)
        t10 = world.scene.add(RigidPrim(prim_path=t10_prim_path,
                                          name="table_w10",
                                          position=[4, -1, 0],
                                          orientation=[1, 0, 0, 1],
                                          scale=[0.005]))



        # Station W8
        t8_prim_path = "/World/table_w8"
        t8_absolute_asset_path = "/home/panda/total_assembly_tender_reducedsize_v2.usd"
        add_reference_to_stage(usd_path=t8_absolute_asset_path, prim_path=t8_prim_path)
        t8 = world.scene.add(RigidPrim(prim_path=t8_prim_path,
                                          name="table_w8",
                                          position=[3, -2, 0],
                                          scale=[0.005]))
        t8_2_prim_path = "/World/table_w8_2"
        t8_2_absolute_asset_path = "/home/panda/total_assembly_tender_reducedsize_v2.usd"
        add_reference_to_stage(usd_path=t8_2_absolute_asset_path, prim_path=t8_2_prim_path)
        t8_2 = world.scene.add(RigidPrim(prim_path=t8_2_prim_path,
                                          name="table_w8_2",
                                          position=[2.3, -2, 0],
                                          scale=[0.005]))

        # add_reference_to_stage(usd_path=absolute_asset_path, prim_path="/World/panda_dummy_w8")
        # panda_dummy_w8 = world.scene.add(Robot(prim_path="/World/panda_dummy_w8", name="panda_dummy_w8", 
        #                                     position = [3.3, -2.2, 0.43],
        #                                     orientation = [1,0,0,-1]))
        # add_reference_to_stage(usd_path=absolute_asset_path, prim_path="/World/panda_dummy_w5")
        # panda_dummy_w5 = world.scene.add(Robot(prim_path="/World/panda_dummy_w5", name="panda_dummy_w5", 
        #                                     position = [2.6, -2.2, 0.43],
        #                                     orientation = [1,0,0,-1]))
        

        # Station W5
        t5_prim_path = "/World/table_w5"
        t5_absolute_asset_path = "/home/panda/basic_table_automatica_v2.usd"
        add_reference_to_stage(usd_path=t5_absolute_asset_path, prim_path=t5_prim_path)
        t5 = world.scene.add(RigidPrim(prim_path=t5_prim_path,
                                          name="table_w5",
                                          position=[1.8, -2.2, 0],
                                          scale=[0.005],
                                          orientation=[1,0, 0, 0]
                                          ))
        
        # Station W7

        # Center 
        
        #Comissioning Robot Darko
        # darko_prim_path = "/World/darko"
        # darko_absolute_asset_path = "/home/panda/darko_automatica_v2.usd"
        # add_reference_to_stage(usd_path=darko_absolute_asset_path, prim_path=darko_prim_path)
        # darko = world.scene.add(RigidPrim(prim_path=darko_prim_path,
        #                                   name="darko",
        #                                   position=[3, -1, 0],
        #                                   scale=[0.7]))
        
        # dp_prim_path = "/World/dp"
        # dp_absolute_asset_path = "/home/panda/Franka/franka.usd"
        # add_reference_to_stage(usd_path=dp_absolute_asset_path, prim_path=dp_prim_path)
        # dp = world.scene.add(RigidPrim(prim_path=dp_prim_path,
        #                                   name="dp",
        #                                   position=[3, -1, 0.5],
        #                                   scale=[0.7]))

        # Background Robots (secondary)
        # W4 -  CURRENTLY NOT VISIBLE
        # col4_prim_path = "/World/col_w4"
        # col4_absolute_asset_path = "/home/panda/collective_automatica_v6.usd"
        # add_reference_to_stage(usd_path=col4_absolute_asset_path, prim_path=col4_prim_path)
        # dcol4p = world.scene.add(RigidPrim(prim_path=col4_prim_path,
        #                                   name="col_w4",
        #                                   position=[7, 1, 0],
        #                                   orientation = [1,1,0,0],
        #                                   scale=[1]))
        # # W11
        # col11_prim_path = "/World/col_w11"
        # col11_absolute_asset_path = "/home/panda/collective_automatica_v2.usd"
        # add_reference_to_stage(usd_path=col11_absolute_asset_path, prim_path=col11_prim_path)
        # col11 = world.scene.add(RigidPrim(prim_path=col11_prim_path,
        #                                   name="col_w11",
        #                                   position=[6, -3, 0],
        #                                   orientation = [1,1,0,0],
        #                                   scale=[0.7]))



        ########



        # Add a cube for collision 
        # cube_prim_path = "/World/secu_cube"
        # secu_cube = world.scene.add(RigidPrim(prim_path=cube_prim_path, 
        #                                           name="secu_cube", 
        #                                           position=[0.4, 0.3, 0.6], 
        #                                           scale=[0.1, 0.2, 0.1], 
        #                                           color=np.array([1.0, 0.99, 0.0])))


        panda_robot = world.scene.add(Robot(prim_path="/World/Panda_Robot", name="panda_robot", 
                                            position = [0, 0, 0.45],
                                            orientation = [1,0,0,-1]))
        

        print("Added robot to the scene")
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        print("Num of degrees of freedom before first reset: " + str(panda_robot.num_dof)) # prints None
        
        
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("panda_robot")
        
        self._casing = self._world.scene.get_object("casing") 
        self._ring = self._world.scene.get_object("ring")
        self._table = self._world.scene.get_object("table")

        print("------ Panda Infos ------")
        print("  DOF-Names     : ", self._franka.dof_names)
        print("  DOF-Properties: ", self._franka.dof_properties)

        print("---- Set-Up Logger ----")
        filepath = os.path.join("/home/panda", "test_node2_full_integration.txt")
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
