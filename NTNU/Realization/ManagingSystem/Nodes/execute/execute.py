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

class execute(Node):

    def __init__(self, config='config.yaml',verbose=True):
        super().__init__(config=config,verbose=verbose)

        self._name = "execute"
        self.logger.info("execute instantiated")

        #<!-- cc_init START--!>
        # user includes here
        #<!-- cc_init END--!>

    # -----------------------------AUTO-GEN SKELETON FOR executer-----------------------------
    def executer(self,msg):
        plan = self.knowledge.read("plan",queueSize=1)
        _predictedPath = predictedPath()

        #<!-- cc_code_executer START--!>

        # user code here for executer

        _predictedPath._Confidence= "SET VALUE"    # datatype: Float_32
        _predictedPath._waypoints= "SET VALUE"    # datatype: String

        #<!-- cc_code_executer END--!>

        self.publish_event(event_key='pathEstimate')    # LINK <outport> pathEstimate

    def register_callbacks(self):
        self.register_event_callback(event_key='plan', callback=self.executer)        # LINK <inport> plan

def main(args=None):

    node = execute(config='config.yaml')
    node.register_callbacks()
    node.start()

if __name__ == '__main__':
    main()
    try:
       while True:
           time.sleep(1)
    except:
       exit()