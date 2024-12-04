# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
from rpio.clientLibraries.rpclpy.node import Node
import time

try:
    from .messages import *
except (ValueError, ImportError):
    from messages import *

#<!-- cc_include START--!>
import random

#<!-- cc_include END--!>

#<!-- cc_code START--!>
# user code here
#<!-- cc_code END--!>

class Plan(Node):

    def __init__(self, config='config.yaml',verbose=True):
        super().__init__(config=config,verbose=verbose)

        self._name = "Plan"
        self.logger.info("Plan instantiated")

        #<!-- cc_init START--!>
        # user includes here
        #<!-- cc_init END--!>

    # -----------------------------AUTO-GEN SKELETON FOR planner-----------------------------
    def planner(self,msg):
        ship_status = self.knowledge.read("ship_status",queueSize=1)
        _NewModel = NewModel()

        #<!-- cc_code_planner START--!>

        # List of available models
        available_models = ["ShipModel_M1", "ShipModel_M2", "ShipModel_M7", "ShipModel_M12", "ShipModel_MS"]

        filtered_models = [s for s in available_models if s != ship_status._ship_predistion_model]

        _NewModel._ship_prediction_model = random.choice(filtered_models)


        #<!-- cc_code_planner END--!>

        _success = self.knowledge.write(cls=_NewModel)
        self.publish_event(event_key='new_plan')    # LINK <outport> new_plan

    def register_callbacks(self):
        self.register_event_callback(event_key='anomaly', callback=self.planner)     # LINK <eventTrigger> anomaly
        self.register_event_callback(event_key='anomaly', callback=self.planner)        # LINK <inport> anomaly

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