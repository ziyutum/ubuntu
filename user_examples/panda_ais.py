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
# from omni.physx.scripts import PhysxUtils
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
import omni.usd
from pxr import Usd, Sdf
from pxr import UsdGeom
import omni.isaac.core.utils.prims as prim_utils
from pxr import Gf

physx = acquire_physx_interface()

physx.overwrite_gpu_setting(1) # 1 means using GPU


GRAD_TO_RAD = 2*pi/360
HOST_ADDR = "127.0.0.1"
# PORT = 32323 # Port of the Azure Client App (or the simulation)
PORT = 8080
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
        self.last_joint_positions = np.array([0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4, 0.04, 0.04])
        self.previous_angles = [0, 0, 0] 
        

        return

    def setup_scene(self):
        # 1: Set up world scene 
        world = self.get_world()
        print("[INFO] Set up world scene.")
        #world.scene.add_default_ground_plane()
        world.scene.add(GroundPlane(
            prim_path="/World/groundPlane", 
            size=0.25,
            color=np.array([0.5, 0.5, 0.5])))
        print("[INFO] Added groundplane to scene.")

     

        # # 3. Add Franka robot to world scene  
        # absolute_asset_path1 = "/home/masais/panda/Franka/franka_zy.usd"
        # add_reference_to_stage(usd_path=absolute_asset_path1, prim_path="/World/franka")
        # panda_robot = world.scene.add(Robot(prim_path="/World/franka/franka", name="panda_robot", 
        #                                     # scale =[100],
        #                                     position=[0.23988, -0.7097, 1],
        #                                     # orientation = [1,0,0,-1]
        #                                     ))
        # print("Added robot to the scene")
        # print("Num of degrees of freedom before first reset: " + str(panda_robot.num_dof)) # prints None


        ##ziyu 04.09 commit myjoghurt to franka----------------------------------------------------------------------------------------------------------------------------
        # 3. Add myJoghurt to world scene  -------------------------------------------------------------------------------------------------
        absolute_asset_path = "/home/masais/panda/Franka/myziyu0509.usd"
        # This will create a new XFormPrim and point it to the usd file as a reference
        # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=absolute_asset_path, prim_path="/World/myJoghurt")
        myjoghurt = world.scene.add(RigidPrim(prim_path="/World/myJoghurt/myJoghurt_ZIyu", name="myjoghurt", 
                                            scale = [0.01],
                                            position=[0,0,0.8280]
                                           ))
        
        #------------------------------------------------------------------------------------------------------------------
        
        absolute_asset_pathfranka = "/home/masais/panda/Franka/franka1209.usd"
        # # This will create a new XFormPrim and point it to the usd file as a reference
        # # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=absolute_asset_pathfranka, prim_path="/World/franka")
        

        panda_robot = world.scene.add(Robot(prim_path="/World/franka/factory_franka", name="panda_robot", 
                                     scale =[1],
                                     position=np.array([0.282,-0.70,0.79]),
        #                             # orientation = [1,0,0,-1]
                                    ))
        #franka = world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="panda_robot"))
        
        #-----------------------------------------------------------------------------------------------------
        absolute_asset_pathbottle = "/home/masais/panda/Franka/bottle.usd"
        # # This will create a new XFormPrim and point it to the usd file as a reference
        # # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=absolute_asset_pathbottle, prim_path="/World/bottle")
        

        bottle = world.scene.add(RigidPrim(prim_path="/World/bottle/bottle", name="bottle", 
                                     scale =[0.001,0.001,0.001],
                                     position=[0.74,-0.975,0.801],
        #                             # orientation = [1,0,0,-1]
                                    ))
        #-------------------------------------------------------------------------------------------------------------
        #-----------------------------------------------------------------------------------------------------
        absolute_asset_pathswich1 = "/home/masais/panda/Franka/swich1.usd"
        # # This will create a new XFormPrim and point it to the usd file as a reference
        # # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=absolute_asset_pathswich1, prim_path="/World/switch1")
        

        switch1 = world.scene.add(RigidPrim(prim_path="/World/switch1/Switch1", name="switch1", 
                                     scale =[0.001,0.001,0.001],
                                     position=[-0.2951,0.02238,0.93452],
                                     orientation = [0.69454,0,0,0.71945]
                                    ))
        add_reference_to_stage(usd_path=absolute_asset_pathswich1, prim_path="/World/switch2")
        

        switch2 = world.scene.add(RigidPrim(prim_path="/World/switch2/Switch1", name="switch2", 
                                     scale =[0.001,0.001,0.001],
                                     position=[0.25412,0.02238,0.93452],
                                     orientation = [1,0,0,0]
                                    ))
        add_reference_to_stage(usd_path=absolute_asset_pathswich1, prim_path="/World/switch3")
        

        switch3 = world.scene.add(RigidPrim(prim_path="/World/switch3/Switch1", name="switch3", 
                                     scale =[0.001,0.001,0.001],
                                     position=[0.80317,0.02238,0.93452],
                                     orientation = [1,0,0,0]
                                    ))
        #-------------------------------------------------------------------------------------------------------------
        # absolute_asset_pathbottle = "/home/masais/panda/Franka/bottle.usd"
        # # # This will create a new XFormPrim and point it to the usd file as a reference
        # # # Similar to how pointers work in memory
        # add_reference_to_stage(usd_path=absolute_asset_pathbottle, prim_path="/World/bottle")
        

        # bottle = world.scene.add(RigidPrim(prim_path="/World/bottle/_691296_GLAS_mit_Barcode_26_2574", name="bottle", 
        #                              scale =[0.001],
        #                              position=[0.77,-0.975,0.825],
        # #                             # orientation = [1,0,0,-1]
        #                             ))
        #-------------------------------------------------------------------------------------------------------------------

        # world.scene.add(
        #     DynamicCuboid(
        #         prim_path="/World/random_cube",
        #         name="fancy_cube",
        #         position=np.array([0.765, -0.96, 0.85]),
        #         scale=np.array([0.0515, 0.03, 0.1]), #0.08 in z
        #         color=np.array([0, 0, 1.0]),
        #     )
        # )
        #-------------------------------------------------------------------------------------------------------------

        
        # Create and set PhysicsMaterial for the bottle
        # stage = Usd.Stage.Open(self.get_world().get_stage().GetRootLayer().identifier)
        
        # # Define the physics material with friction
        # physx_material_path = "/World/myJoghurt/myJoghurt_ZIyu/Geometry/VN3066_000_Layout_Uni_Kassel_070919_0/_691296_GLAS_mit_Barcode_30_2578_PhysxMaterial"
        # physx_material = UsdPhysics.Material.Define(stage, physx_material_path)
        # physx_material.CreateStaticFrictionAttr(0.5)
        # physx_material.CreateDynamicFrictionAttr(0.5)
        # physx_material.CreateRestitutionAttr(0.1)

        # # Assign the physics material to the bottle
        # bottle_prim = stage.GetPrimAtPath("/World/myJoghurt/myJoghurt_ZIyu/Geometry/VN3066_000_Layout_Uni_Kassel_070919_0/_691296_GLAS_mit_Barcode_30_2578")
        # if bottle_prim:
        #     UsdPhysics.MaterialBindingAPI.Apply(bottle_prim, physx_material.GetPrim())

        # print("[INFO] Added bottle to the scene with friction properties.")



        
        
        # bottle = world.scene.add(RigidPrim(prim_path="/World/myJoghurt/myJoghurt_ZIyu/Geometry/VN3066_000_Layout_Uni_Kassel_070919_0/_691296_GLAS_mit_Barcode_33_2581",

        #                                                    name="bottle",
        #                                                   #position=[418.74569,-958.25394,-10],
        # #                                                   scale=[1]
        #                                                   ))
    #ziyu end commit----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    #ziyu change myjoghurt to franka testing the gripper
       


        return


    async def setup_post_load(self):
        self._world = self.get_world()
        #self._world.scene.set_gravity([0.0, 0.0, 0.0])
        self._franka = self._world.scene.get_object("panda_robot")
        self._bottle = self._world.scene.get_object("bottle")
        self._cube = self._world.scene.get_object("fancy_cube")
        self._switch1 = self._world.scene.get_object("switch1")
        self._switch2 = self._world.scene.get_object("switch2")
        self._switch3 = self._world.scene.get_object("switch3")
        #self._bottle.set_world_pose(position=[0.5,-1.061,0.7999])
        # self._myjoghurt= self._world.scene.get_object("myjoghurt")
        # initial_joint_positions = [0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.7,0.1,0.1]
        self._franka.set_joint_positions(self.last_joint_positions)
        #----------------------------------------------------------------------------------------------------------------
        # ------------preset switch-----------------------------------------------------------------------
     

        # stage = omni.usd.get_context().get_stage()

        # # 
        # fingertip_path = "/World/franka/factory_franka/panda_fingertip_centered"
        # prim = stage.GetPrimAtPath(fingertip_path)

        
        # if prim.IsValid():
        #     xformable = UsdGeom.Xformable(prim)
        #     transform_matrix = xformable.ComputeLocalToWorldTransform(0)
        #     fingertip_position = transform_matrix.ExtractTranslation()
        #     print("------ finger Infos ------")
        #     print(f"Fingertip XYZ position: {fingertip_position}")
        # else:
        #     print(f"Prim at path {fingertip_path} is not valid!")
            
        # bottle_position = [fingertip_position[0], fingertip_position[1], fingertip_position[2] - 0.2] 
        # self._bottle.set_world_pose(position=bottle_position)
        #-----------------------------------------------------------------------------------------------------------------
        print("------ Panda Infos ------")
        print("  DOF-Names     : ", self._franka.dof_names)
        print("  DOF-Properties: ", self._franka.dof_properties)
        current_bottle_pose = self._bottle.get_world_pose()
        print("Current bottle position: ", current_bottle_pose)

        # position, orientation = self._switch1.get_world_pose()
        # print(f"Current orientation (quaternion): {orientation}")
        # ## Unpack the quaternion (w, x, y, z) and ensure values are Python float type
        # w, x, y, z = float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])

        #     # Create a quaternion object from current orientation (ensure it's Quatf)
        # current_quat = Gf.Quatf(w, Gf.Vec3f(float(x), float(y), float(z)))

        # # Create the new rotation quaternion for 90 degrees counterclockwise around the Z axis
        # new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 80)  # Counterclockwise rotation of 90 degrees
        # new_quat = new_rotation.GetQuat()

        # # Convert new_quat to Quatf if it's of type Quatd
        # if isinstance(new_quat, Gf.Quatd):
        #     new_quat = Gf.Quatf(new_quat.GetReal(), Gf.Vec3f(new_quat.GetImaginary()))

        # # Combine the current orientation with the new rotation
        # final_quat = current_quat * new_quat  # Quaternion multiplication

        # # Set the new quaternion rotation in the format [w, x, y, z]
        # new_orientation = [final_quat.GetReal(), final_quat.GetImaginary()[0], final_quat.GetImaginary()[1], final_quat.GetImaginary()[2]]

        # # Set the new pose (keep the position unchanged, only update the rotation)
        # self._switch1.set_world_pose(position, new_orientation)
        # print("switch1 has been rotated 90 degrees counterclockwise around the Z axis.")

        # # Print the new orientation after rotation
        # print(f"New orientation (quaternion) after rotation: {new_orientation}")


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

        ## ziyu manuell apply action
        # self._franka.apply_action(control_actions = ArticulationAction(
        #             joint_positions=[0.285453,-0.246976,0.407148,-2.135107,0.033087,1.897359,-0.401687,0.05,0.05],
        #             joint_efforts=None,
        #             joint_velocities=None
        #         ))
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
       ## end modify
        #self._world.add_physics_callback("sending_actions", callback_fn=self.send_bottle_actions)
        return
    
    def log_joint_position_during_sim(self, step_size):
        self._logger.write("\n{t} \nJoint-Pos.: {pos}\nJoint-Vel.: {vel}\nSoll: {d}".format(t=datetime.datetime.now(), 
                                                                                 pos=self._franka.get_joint_positions(), 
                                                                                 vel=self._franka.get_joint_velocities(),
                                                                                 d=self._jointPos1))
        return
   
    


    def send_robot_actions(self, step_size): 

        #----------------------------------------------------------------------------------------------------------------------------------
            
        # position, orientation = self._switch1.get_world_pose()
        # print(f"Current orientation (quaternion): {orientation}")
        # ## Unpack the quaternion (w, x, y, z) and ensure values are Python float type
        # w, x, y, z = float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])

        #     # Create a quaternion object from current orientation (ensure it's Quatf)
        # current_quat = Gf.Quatf(w, Gf.Vec3f(float(x), float(y), float(z)))

        # # Create the new rotation quaternion for 90 degrees counterclockwise around the Z axis
        # new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)  # Counterclockwise rotation of 90 degrees
        # new_quat = new_rotation.GetQuat()

        # # Convert new_quat to Quatf if it's of type Quatd
        # if isinstance(new_quat, Gf.Quatd):
        #     new_quat = Gf.Quatf(new_quat.GetReal(), Gf.Vec3f(new_quat.GetImaginary()))

        # # Combine the current orientation with the new rotation
        # final_quat = current_quat * new_quat  # Quaternion multiplication

        # # Set the new quaternion rotation in the format [w, x, y, z]
        # new_orientation = [final_quat.GetReal(), final_quat.GetImaginary()[0], final_quat.GetImaginary()[1], final_quat.GetImaginary()[2]]

        # # Set the new pose (keep the position unchanged, only update the rotation)
        # self._switch1.set_world_pose(position, new_orientation)
        # print("switch1 has been rotated 90 degrees counterclockwise around the Z axis.")

        # # Print the new orientation after rotation
        # print(f"New orientation (quaternion) after rotation: {new_orientation}")
                
                
        #----------------------------------------------------------------------------------------------------------------------------------



        try:
            # Clear the receive buffer 
            self._client.setblocking(0)
            chunks = []
            while True:
                try:
                    chunk = self._client.recv(100)
                    if not chunk:
                        break
                    chunks.append(chunk)
                except socket.error:
                    break
            self._client.setblocking(1)
            
            data = b''.join(chunks)
            if not data:
                # print("No data received from client.")
                # print(self._franka.get_applied_action())
                #self._franka.set_joint_positions(self.last_joint_positions)
                # self._franka.apply_action(control_actions=ArticulationAction(
                #     joint_positions=self.last_joint_positions,
                #     joint_efforts=None, # [0.0] * len(self._last_joint_positions),  # Keep efforts at zero
                #     joint_velocities=None#[0.0] * len(self._last_joint_positions)  # Keep velocities at zero
                #     ))
                # self._franka.apply_action(control_actions=ArticulationAction(
                #     joint_positions=np.array([0.0, 0.0]), 
                #     joint_indices=np.array([7, 8]),
                #     joint_efforts=None, # [0.0] * len(self._last_joint_positions),  # Keep efforts at zero
                #     joint_velocities=None#[0.0] * len(self._last_joint_positions)  # Keep velocities at zero
                #     ))


                return
            #     print("No data received from client.")
            #     # #ziyu added if ther eis no data received, just keep the pose loike the last position
            #     # # if hasattr(self, '_last_joint_positions'):
            #     # #     print("No new data received, keeping the last joint positions.")
            #     # if self._last_joint_positions is not None:
            #     #     print("No new data received, keeping the last joint positions.")
            #     #print(self.last_joint_positions)
            #     #self._franka.set_joint_positions(self.last_joint_positions)
                
            # # else:
            # #     print("No data received and no previous joint positions available.")
            #     return

            # Only process the last complete packet
            messages = data.decode('utf-8').strip().split('\n')
            last_message = messages[-1]

            print("Read data: ", last_message)

            # JZh added update_count_sum to count total number of data received from RCP
            # self.update_count_sum=self.update_count_sum+1
            # print(self.update_count_sum)
            # end

            json_list_buffer = json.loads(last_message)
            nparray = np.array(json_list_buffer['q'])            
            #nparray = np.array(json_list_buffer)
            
            print("nparray: ", nparray)
            print("Array size:", nparray.size)

            if nparray.size == 9:
                nparray_new = nparray[0:9]  # Take the first 7 elements for joint positions
                self._franka.set_joint_positions(nparray_new)
                self.last_joint_positions[:9]= nparray_new[:9]
                # get current time
                now = datetime.datetime.now()
                # print current time
                print("Current local time:", now.strftime("%Y-%m-%d %H:%M:%S:%f")[:-3])
                #------------------------------------------------------------------------------------------------------------------------------------
                last_two_elements = nparray_new[-2:]
                if np.logical_and(last_two_elements >= 0.0194, last_two_elements <= 0.0205).all():
                    # Get the fingertip position
                    stage = omni.usd.get_context().get_stage()
                    fingertip_path = "/World/franka/factory_franka/panda_fingertip_centered"
                    prim = stage.GetPrimAtPath(fingertip_path)

                    if prim.IsValid():
                        xformable = UsdGeom.Xformable(prim)
                        transform_matrix = xformable.ComputeLocalToWorldTransform(0)
                        fingertip_position = transform_matrix.ExtractTranslation()
                        bottle_position = [fingertip_position[0], fingertip_position[1], fingertip_position[2] - 0.05]
                        self._bottle.set_world_pose(position=bottle_position)
                #------------------------------------------------------------------------------------------------------------------------------------------------------
                   #switch1_prim =stage.GetPrimAtPath("/World/myJoghurt/myJoghurt_ZIyu/Geometry/VN3066_000_Layout_Uni_Kassel_070919_0/VN3066_020_Band_Layout_1_91/VN3066_010_Weiche_3_413")
            

                else:
                    # If last two elements are not both 0.02, keep bottle's position unchanged
                    print("Bottle position remains unchanged")












                #------------------------------------------------------------------------------------------------------

                # self._franka.apply_action(control_actions = ArticulationAction(
                #     joint_positions=nparray_new,
                #     joint_efforts=None,#[0, 0, 0, 0, 0, 0, 0, 20, 20],
                #     joint_velocities=None
                # ))
            # elif nparray.size == 7:
            #     nparray_new = np.append(nparray, [0, 0])
            #     self.last_joint_positions= nparray_new
            #     self._franka.set_joint_positions(nparray_new)
            elif nparray.size == 2:
                nparray_new = nparray[0:2]
                bottle_position = np.array([nparray_new[0], nparray_new[1], 0.91567])
                self._bottle.set_world_pose(position=bottle_position)
            elif nparray.size == 3:
                nparray_new = nparray[0:3]
                switches = [self._switch1, self._switch2, self._switch3]  # 

                for i, switch in enumerate(switches):
                    position, orientation = switch.get_world_pose()
                    

                    # Unpack the quaternion (w, x, y, z)
                    w, x, y, z = map(float, orientation)

                    # Create a quaternion object for the current switch
                    current_quat = Gf.Quatf(w, Gf.Vec3f(x, y, z))
                    received_angle =  float(nparray_new[i])
                    rotation_angle = received_angle-self.previous_angles[i] # 

                    # Create new rotation quaternion for the current switch
                    new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), rotation_angle)  # Around Z-axis
                    new_quat = new_rotation.GetQuat()

                    # Convert to Quatf if necessary
                    if isinstance(new_quat, Gf.Quatd):
                        new_quat = Gf.Quatf(new_quat.GetReal(), Gf.Vec3f(new_quat.GetImaginary()))

                    # Combine the current orientation with the new rotation
                    final_quat = current_quat * new_quat

                    # Set new orientation in the format [w, x, y, z]
                    new_orientation = [final_quat.GetReal(), final_quat.GetImaginary()[0], final_quat.GetImaginary()[1], final_quat.GetImaginary()[2]]
                    
                    # Set the new pose (keep the position unchanged)
                    switch.set_world_pose(position, new_orientation)
                    self.previous_angles[i] = received_angle


                    #----------------------
                     # Get the stage and find the corresponding hole for each switch
                    if rotation_angle != 0: 
                        stage = omni.usd.get_context().get_stage()
                        hole_path = f"/World/switch{i+1}/Switch1/VN3066_010_Weiche_3_413/swich1holecenter"

                        # Print the hole path for debugging purposes
                        print(f"Processing switch{i+1}, hole path: {hole_path}")

                        # Ensure the hole prim exists for the current switch
                        prim = stage.GetPrimAtPath(hole_path)

                        if prim.IsValid():
                            xformable = UsdGeom.Xformable(prim)
                            transform_matrix = xformable.ComputeLocalToWorldTransform(0)
                            hole_position = transform_matrix.ExtractTranslation()

                            # Print the hole position for debugging purposes
                            print(f"Switch{i+1} hole position: {hole_position}")

                            # Update the bottle position based on the current switch's hole position
                            bottle_position = [hole_position[0], hole_position[1], hole_position[2] - 0.03]
                            print(f"Setting bottle position: {bottle_position}")  # Debug bottle position
                            self._bottle.set_world_pose(position=bottle_position)
            #------------------------


                    # Generate dynamic hole path based on the current switch (switch1, switch2, switch3)
                    # stage = omni.usd.get_context().get_stage()
                    #hole_path = f"/World/switch{i+1}/Switch1/VN3066_010_Weiche_3_413/swich1holecenter"
                     # Dynamically generate hole path based on switch index
                      # Determine hole path dynamically based on switch index
                    # hole_paths = [
                    #     "/World/switch1/Switch1/VN3066_010_Weiche_3_413/swich1holecenter",
                    #     "/World/switch2/Switch1/VN3066_010_Weiche_3_413/swich1holecenter",
                    #     "/World/switch3/Switch1/VN3066_010_Weiche_3_413/swich1holecenter"
                    # ]
                    
                    # hole_path = hole_paths[i]
                    # print(hole_path)
                    # prim = stage.GetPrimAtPath(hole_path)
                    # print(prim)

                    # if prim.IsValid():
                    #     xformable = UsdGeom.Xformable(prim)
                    #     transform_matrix = xformable.ComputeLocalToWorldTransform(0)
                    #     hole_position = transform_matrix.ExtractTranslation()

                    #     # Adjust the bottle position based on hole position
                    #     bottle_position = [hole_position[0], hole_position[1], hole_position[2] - 0.03]
                    #     self._bottle.set_world_pose(position=bottle_position)
                    #------------------------------------------------
                    # if i == 0:
                    #     hole_path = "/World/switch1/Switch1/VN3066_010_Weiche_3_413/swich1holecenter"
                    #     prim = stage.GetPrimAtPath(hole_path)

                    #     if prim.IsValid():
                    #         xformable = UsdGeom.Xformable(prim)
                    #         transform_matrix = xformable.ComputeLocalToWorldTransform(0)
                    #         hole_position = transform_matrix.ExtractTranslation()

                    #         # Adjust the bottle position
                    #         bottle_position = [hole_position[0], hole_position[1], hole_position[2] - 0.03]
                    #         self._bottle.set_world_pose(position=bottle_position)

                    # elif i == 1:
                    #     hole_path = "/World/switch2/Switch1/VN3066_010_Weiche_3_413/swich1holecenter"
                    #     prim = stage.GetPrimAtPath(hole_path)

                    #     if prim.IsValid():
                    #         xformable = UsdGeom.Xformable(prim)
                    #         transform_matrix = xformable.ComputeLocalToWorldTransform(0)
                    #         hole_position = transform_matrix.ExtractTranslation()

                    #         # Adjust the bottle position
                    #         bottle_position = [hole_position[0], hole_position[1], hole_position[2] - 0.03]
                    #         self._bottle.set_world_pose(position=bottle_position)
                    # elif i == 2:
                    #     hole_path = "/World/switch3/Switch1/VN3066_010_Weiche_3_413/swich1holecenter"
                    #     prim = stage.GetPrimAtPath(hole_path)

                    #     if prim.IsValid():
                    #         xformable = UsdGeom.Xformable(prim)
                    #         transform_matrix = xformable.ComputeLocalToWorldTransform(0)
                    #         hole_position = transform_matrix.ExtractTranslation()

                    #         # Adjust the bottle position
                    #         bottle_position = [hole_position[0], hole_position[1], hole_position[2] - 0.03]
                    #         self._bottle.set_world_pose(position=bottle_position)
                            
                    #--------------------------------------------------

                    #/World/switch2/Switch1/VN3066_010_Weiche_3_413/swich1holecenter

                    # Ensure the hole prim exists
                    # prim = stage.GetPrimAtPath(hole_path)

                    # if prim.IsValid():
                    #     xformable = UsdGeom.Xformable(prim)
                    #     transform_matrix = xformable.ComputeLocalToWorldTransform(0)
                    #     hole_position = transform_matrix.ExtractTranslation()

                    #     # Adjust the bottle position
                    #     bottle_position = [hole_position[0], hole_position[1], hole_position[2] - 0.03]
                    #     self._bottle.set_world_pose(position=bottle_position)
                





                
                

            else:
                print("Skipped array of incompatible size: ", nparray.size)
            
            self._logger.write("\n{t} Joint-Pos.: {pos}".format(t=datetime.datetime.now(), 
                                                                pos=nparray_new))

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