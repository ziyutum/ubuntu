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
import datetime #from datetime import datetime
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

import omni.physxdemos as demo
from omni.isaac.core.objects import DynamicCuboid

from omni.physx import acquire_physx_interface
import omni.isaac.core.utils.prims as prim_utils

physx = acquire_physx_interface()

physx.overwrite_gpu_setting(1) # 1 means using GPU

GRAD_TO_RAD = 2*pi/360
HOST_ADDR = "127.0.0.1"
PORT = 8080 # Port of the Azure Client App (or the simulation)

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

class Robot2Isaac(BaseSample):
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
        
        self.update_count_sum = 0 # JZh: added update_count_sum to count total number of data received from RCP
        return

    def setup_scene(self):
        # 1: Set up world scene 
        world = self.get_world()
        print("[INFO] Set up world scene.")
        # world.scene.add_default_ground_plane()

        # 2. Add groundplane to world scene 
        world.scene.add(GroundPlane(
            prim_path="/World/groundPlane", 
            size=25,
            color=np.array([0.5, 0.5, 0.5])))
        print("[INFO] Added groundplane to scene.")
        
        #todo: need debug
        # # Define the friction material
        # friction_material = world.scene.add(RigidPrim(
        #     prim_path="/World/FrictionMaterial",
        #     name="friction_material",
        #     mass=0.0032,
        #     position=[0.36, -0.3, 0.42],
        #     scale=[0.5, 0.5, 0.18],
        #     material={
        #         "static_friction": 0.8,  # Example value
        #         "dynamic_friction": 0.5  # Example value
        #     }
        # ))


        # # 3. Add panda to world scene  
        # absolute_asset_path = "/home/panda/Franka/franka.usd"
        panda_prim_path="/World/Panda_Robot"
        # absolute_asset_path = "/home/panda/Desktop/USDs/franka_init.usd"
        absolute_asset_path1 = "/home/masais/panda/Franka/franka_zy.usd"
        # This will create a new XFormPrim and point it to the usd file as a reference
        # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=absolute_asset_path1, prim_path=panda_prim_path)
        panda_robot = world.scene.add(Robot(prim_path="/World/Panda_Robot", name="panda_robot", 
                                    position = [0, 0, 0.43], # Feng Xu 12.07.2024 0.45 to 0.43
                                    orientation = [1,0,0,-1]))
        print("[INFO] Added panda to scene.")
        
        #panda_robot=world.scene.add(Franka(prim_path="/World/panda_robot",name="panda_robot"))



        # # 4. Add Mir to world scene 
        # # Feng Xu 12.07.2024
        # mir_prim_path = "/World/Mir_basement"
        # #casing_absolute_asset_path = "/home/panda/Mir_casting.usd"
        # mir_absolute_asset_path = "/home/panda/Mir_casing_addfriction.usd"  # Feng Xu 12.07.2024
        # add_reference_to_stage(usd_path=mir_absolute_asset_path, prim_path=mir_prim_path)

        # mir = world.scene.add(RigidPrim(prim_path=mir_prim_path,
        #                                    name="Mir_basement",
        #                                    position=[0.67, -0.45, 0.01], #fxu init[0.56, -0.42, 0.1]
        #                                    scale=[0.01051]))  #fxu init[0.008] adjust for testing
        # print("[INFO] Added Mir basement to scene.")
        
        # # 5. Add the casing to the scene
        # casing_prim_path = "/World/casing"
        # #casing_absolute_asset_path = "/home/panda/Mir_casting_basement.usd"
        # casing_absolute_asset_path = "/home/panda/casing.usd"
        # add_reference_to_stage(usd_path=casing_absolute_asset_path, prim_path=casing_prim_path)

        # casing = world.scene.add(RigidPrim(prim_path=casing_prim_path,
        #                                    name="casing",
        #                                    position=[0.3186, -0.3675, 0.48],
        #                                    scale=[0.008]))
        # print("[INFO] Added casing to scene.")

        # # add. Add a base(cube) to the scene
        # base_prim_path = "/World/base1"
        # base_absolute_asset_path = "/home/panda/Desktop/USDs/base.usd"
        # add_reference_to_stage(usd_path=base_absolute_asset_path, prim_path=base_prim_path)
        # # position=[0.35, 0.3, 0.6] sollte sein
        # base = world.scene.add(RigidPrim(
        # prim_path=base_prim_path,
        # name="base1",
        # mass=0.0032,
        # position=[0.36, -0.3, 0.39],
        # scale=[0.5, 0.5, 0.5],
        # # material=friction_material
        # ))
        # scale=[0.1, 0.08, 0.3] # position=[0.295, 0.29, 1] # Feng Xu 12.07.2024 1. RigidPrim to GroundPlane 2. [0.5, 0.3, 0.6] to [0.3, 0.3, 0.63]
        


        # # 6. Add the design-ring to the scene
        # ring_prim_path = "/World/ring"
        # ring_absolute_asset_path = "/home/panda/designring.usd"
        # add_reference_to_stage(usd_path=ring_absolute_asset_path, prim_path=ring_prim_path)
        # # position=[0.35, 0.3, 0.6] sollte sein
        # ring = world.scene.add(RigidPrim(prim_path=ring_prim_path,
        #                                    name="ring",
        #                                    mass = (0.0032),
        #                                    position=[0.295, 0.2845, 0.65], # Feng Xu 12.07.2024 1. RigidPrim to GroundPlane 2. [0.5, 0.3, 0.6] to [0.3, 0.3, 0.63]
        #                                    scale=[0.007])) #ini: 0.017
        
        # # 7. Add a base(cube) to the scene
        # base_prim_path = "/World/base2"
        # base_absolute_asset_path = "/home/panda/Desktop/USDs/base.usd"
        # add_reference_to_stage(usd_path=base_absolute_asset_path, prim_path=base_prim_path)
        # # position=[0.35, 0.3, 0.6] sollte sein
        # base = world.scene.add(RigidPrim(prim_path=base_prim_path,
        #                                    name="base2",
        #                                    mass = (0.0032),
        #                                    position=[0.295, 0.2845, 0.615], # position=[0.295, 0.29, 1] # Feng Xu 12.07.2024 1. RigidPrim to GroundPlane 2. [0.5, 0.3, 0.6] to [0.3, 0.3, 0.63]
        #                                    scale=[0.1, 0.1, 0.181])) # scale=[0.1, 0.08, 0.3]
    
        # world.scene.add(
        # DynamicCuboid(
        #     prim_path="/World/random_cube",
        #     name="fancy_cube",
        #     position=np.array([0.295, 0.2845, 1]),
        #     scale=np.array([0.05, 0.05, 0.08]),
        #     color=np.array([0, 0, 1.0]),
        #     mass = (0.01),
        #     )
        # )

        # Feng Xu 12.07.2024
        # Friction Setup

    
        print("[INFO] Added design ring to scene.")
        
        # Feng Xu 12.07.2024
        table_prim_path = "/World/table"
        table_absolute_asset_path = "/home/panda/Downloads/CAD_to_USD/manipulation_module_cad_141511/manipulation_module_cad/total_assembly_tender_reducedsize.usd"
        add_reference_to_stage(usd_path=table_absolute_asset_path, prim_path=table_prim_path)
        table = world.scene.add(RigidPrim(prim_path=table_prim_path,
                                          name="table",
                                          position=[0, 1, 0],
                                          scale=[0.007]))
        # change scale to 0.005 in automatica, the original one in DM-demo is 0.007

        # panda_robot = world.scene.add(Robot(prim_path="/World/Panda_Robot", name="panda_robot", 
        #                                     position = [0, 0, 0.43], # Feng Xu 12.07.2024 0.45 to 0.43
        #                                     orientation = [1,0,0,-1]))
        # print("Added robot to the scene")

        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there

        #print("Num of degrees of freedom before first reset: " + str(panda_robot.num_dof)) # prints None
        
        
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
        
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # self._world.add_physics_callback("sending_actions", callback_fn=self.send_mir_actions)
        return

    
    def send_robot_actions(self, step_size): 

        # self._franka.apply_action(control_actions = ArticulationAction(
        #             joint_positions=[0,0,0,0,0,0,0,-0.01,-0.05],
        #             joint_efforts=None,
        #             joint_velocities=None
        #         ))
        # sleep(500)
        # self._franka.apply_action(control_actions = ArticulationAction(
        #             joint_positions=[1,1,1,1,1,1,1,-0.01,-0.01],
        #             joint_efforts=None,
        #             joint_velocities=None
        #         ))


        try:
            # Clear the receive buffer 
            self._client.setblocking(0)
            chunks = []
            while True:
                try:
                    chunk = self._client.recv(4096)
                    if not chunk:
                        break
                    chunks.append(chunk)
                except socket.error:
                    break
            self._client.setblocking(1)
            
            data = b''.join(chunks)
            if not data:
                return

            # Only process the last complete packet
            messages = data.decode('utf-8').strip().split('\n')
            last_message = messages[-1]

            print("Read data: ", last_message)

            # JZh added update_count_sum to count total number of data received from RCP
            self.update_count_sum=self.update_count_sum+1
            print(self.update_count_sum)
            # end

            json_list_buffer = json.loads(last_message)
            nparray = np.array(json_list_buffer)
            print("nparray: ", nparray)
            print("Array size:", nparray.size)

            if nparray.size == 9:
                nparray_new = nparray[0:9]  # Take the first 7 elements for joint positions
                # self._franka.set_joint_positions(nparray_new)
                
                # get current time
                now = datetime.datetime.now()
                # print current time
                print("Current local time:", now.strftime("%Y-%m-%d %H:%M:%S:%f")[:-3])

                self._franka.apply_action(control_actions = ArticulationAction(
                    joint_positions=nparray_new,
                    joint_efforts=None,
                    joint_velocities=None
                ))
            elif nparray.size == 7:
                nparray_new = np.append(nparray, [0, 0])
                self._franka.set_joint_positions(nparray_new)
            else:
                print("Skipped array of incompatible size: ", nparray.size)
            
            self._logger.write("\n{t} Joint-Pos.: {pos}".format(t=datetime.datetime.now(), 
                                                                pos=nparray_new))

        except Exception as ex:
            print(f"Exception while converting data: {ex}")    

        return
    # Feng Xu 08.07.2024;
    # What has changed: 1. nparray[0:0:6] -> nparray[0:9] set_joint_positions should be with 9 elements; 2. nparray_new united; 
    # 3. self._logger.write out of if-elif , in this case making it record at any time. 4. Clear the receive buffer and Only process the last complete packet.
    
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

