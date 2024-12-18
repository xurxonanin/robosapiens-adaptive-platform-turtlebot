# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from rpio.clientLibraries.rpclpy.node import Node
from .messages import *
import time
#<!-- cc_include START--!>
from fractions import Fraction
from .lidarocclusion.masks import BoolLidarMask
from .lidarocclusion.sliding_lidar_masks import sliding_lidar_mask, sliding_prob_lidar_mask
from typing import List, Tuple, Dict
import traceback
import json
import numpy as np
import pickle
import portion
#<!-- cc_include END--!>

#<!-- cc_code START--!>

### USER Defined Functions

#<!-- cc_code END--!>

class Plan(Node):

    def __init__(self, config='config.yaml',verbose=True):
        super().__init__(config=config,verbose=verbose)

        self._name = "Trustworthiness"
        self.logger.info("Trustworthiness instantiated")

        #<!-- cc_init START--!>



        #<!-- cc_init END--!>

    # -----------------------------AUTO-GEN SKELETON FOR planner-----------------------------
    def planner(self,msg):
        _NewPlanMessage = NewPlanMessage()
        _Direction = Direction()

        #<!-- cc_code_planner START--!>

        # user code here for planner


        #<!-- cc_code_planner END--!>

        # _success = self.knowledge.write(cls=_NewPlanMessage)
        # _success = self.knowledge.write(cls=_Direction)

    def register_callbacks(self):
        self.register_event_callback(event_key='anomaly', callback=self.planner)     # LINK <eventTrigger> anomaly
        # self.register_event_callback(event_key='anomaly', callback=self.planner)        # LINK <inport> anomaly

def main(args=None):

    node = Plan(config='config.yaml')
    node.register_callbacks()
    node.start()

if __name__ == '__main__':
    main()
    try:
       while True:
           time.sleep(1)
    except:
       exit()