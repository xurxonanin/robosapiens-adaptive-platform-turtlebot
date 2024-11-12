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
# user includes here
#<!-- cc_include END--!>

#<!-- cc_code START--!>
# user code here
#<!-- cc_code END--!>

class monitor(Node):

    def __init__(self, config='config.yaml',verbose=True):
        super().__init__(config=config,verbose=verbose)

        self._name = "monitor"
        self.logger.info("monitor instantiated")

        #<!-- cc_init START--!>
        # user includes here
        #<!-- cc_init END--!>

    # -----------------------------AUTO-GEN SKELETON FOR shipPoseEstimation-----------------------------
    def shipPoseEstimation(self,msg):
        weatherConditions = self.knowledge.read("weatherConditions",queueSize=1)
        shipPose = self.knowledge.read("shipPose",queueSize=1)
        shipAction = self.knowledge.read("shipAction",queueSize=1)
        _predictedPath = predictedPath()

        #<!-- cc_code_shipPoseEstimation START--!>

        # user code here for shipPoseEstimation

        _predictedPath._Confidence= "SET VALUE"    # datatype: Float_32
        _predictedPath._waypoints= "SET VALUE"    # datatype: String

        #<!-- cc_code_shipPoseEstimation END--!>

        self.publish_event(event_key='pathEstimate')    # LINK <outport> pathEstimate

    def register_callbacks(self):
        self.register_event_callback(event_key='weatherConditions', callback=self.shipPoseEstimation)        # LINK <inport> weatherConditions
        self.register_event_callback(event_key='shipPose', callback=self.shipPoseEstimation)        # LINK <inport> shipPose
        self.register_event_callback(event_key='shipAction', callback=self.shipPoseEstimation)        # LINK <inport> shipAction

def main(args=None):

    node = monitor(config='config.yaml')
    node.register_callbacks()
    node.start()

if __name__ == '__main__':
    main()
    try:
       while True:
           time.sleep(1)
    except:
       exit()