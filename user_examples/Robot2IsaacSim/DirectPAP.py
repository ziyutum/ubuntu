# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Feng Xu 01.08.2024

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

from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController

from pxr import UsdGeom # for the light

physx = acquire_physx_interface()

physx.overwrite_gpu_setting(1) # 1 means using GPU

class DirectPAP(BaseSample):
    def __init__(self) -> None:
        super().__init__()
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
        
        franka = world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka",position = [0, 0, 0], orientation = [1,0,0,-1]))
        

        # 5. Add the casing to the scene
        casing_prim_path = "/World/casing"
        #casing_absolute_asset_path = "/home/panda/Mir_casting_basement.usd"
        casing_absolute_asset_path = "/home/panda/casing.usd"
        add_reference_to_stage(usd_path=casing_absolute_asset_path, prim_path=casing_prim_path)

        casing = world.scene.add(RigidPrim(prim_path=casing_prim_path,
                                           name="casing",
                                           position=[0.3, -0.3, 0.3],
                                           scale=[0.008]))
        print("[INFO] Added casing to scene.")

        # 6. Add the design-ring to the scene
        ring_prim_path = "/World/ring"
        ring_absolute_asset_path = "/home/panda/designring.usd"
        add_reference_to_stage(usd_path=ring_absolute_asset_path, prim_path=ring_prim_path)
        # position=[0.35, 0.3, 0.6] sollte sein
        ring = world.scene.add(RigidPrim(prim_path=ring_prim_path,
                                           name="ring",
                                           position=[0.3, 0.3, 0.3], # Feng Xu 12.07.2024 1. RigidPrim to GroundPlane 2. [0.5, 0.3, 0.6] to [0.3, 0.3, 0.63]
                                           scale=[0.007])) #ini: 0.017
        

    
        print("[INFO] Added design ring to scene.")

    async def setup_post_load(self):
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
        self._base = self._world.scene.get_object("base")
        # Initialize a pick and place controller
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        # World has pause, stop, play..etc
        # Note: if async version exists, use it in any async function is this workflow
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return
    

    async def setup_post_reset(self):
        self._controller.reset()
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return


    def physics_step(self, step_size):
        current_joint_positions = self._franka.get_joint_positions()
        actions = self._controller.forward(
            picking_position=[0.3, 0.3, 0.0003],
            placing_position=[0.3, -0.3, 0.1],
            current_joint_positions=current_joint_positions,
        )

        self._franka.apply_action(actions)

        if self._controller.is_done():
            self._world.pause()
        return