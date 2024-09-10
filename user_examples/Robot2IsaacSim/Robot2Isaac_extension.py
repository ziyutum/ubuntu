# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

#Feng Xu 09.08.2024

import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.user_examples import Robot2Isaac


class Robot2Isaac_extension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="Robot2Isaac",
            title="",
            doc_link="TODO:DOCLINK",
            overview="",
            file_path=os.path.abspath(__file__),
            sample=Robot2Isaac(),
        )
        return
