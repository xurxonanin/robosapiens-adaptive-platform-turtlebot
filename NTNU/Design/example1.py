from rpio.metamodels.aadl2_IL.aadl2_IL import *



def example():
    #-----------------------------------------------------------------------------------------------------------------------
    #--------------------------------------------- MESSAGES ----------------------------------------------------------------
    #-----------------------------------------------------------------------------------------------------------------------
    # weatherConditions message
    windDirection = data(name="windDirection",dataType="Float_64")
    windSpeed = data(name="windSpeed",dataType="Float_64")
    weatherConditions = message(name="weatherConditions",featureList=[windDirection,windSpeed])

    # shipPose message
    SurgeSpeed = data(name="SurgeSpeed",dataType="Float_64")
    SwaySpeed = data(name="SwaySpeed",dataType="Float_64")
    YawRate = data(name="YawRate",dataType="Float_64")
    RollAngle = data(name="RollAngle",dataType="Float_64")
    RollRate = data(name="RollRate",dataType="Float_64")
    Heading = data(name="Heading",dataType="Float_64")
    x = data(name="x",dataType="Float_64")
    y = data(name="y",dataType="Float_64")
    shipPose = message(name="shipPose",featureList=[SurgeSpeed,SwaySpeed,YawRate,RollAngle,RollRate,Heading,x,y])

    # shipAction message
    RudderAngle = data(name="RudderAngle",dataType="Float_64")
    rpm = data(name="rpm",dataType="Float_32")
    shipAction = message(name="shipAction",featureList=[RudderAngle,rpm])

    # shipAction message
    Confidence = data(name="Confidence",dataType="Float_64")
    Waypoints = data(name="Waypoints",dataType="Float_32")
    predictedPath = message(name="predictedPath",featureList=[Confidence,Waypoints])

    # anomaly message
    Anomaly = data(name="Anomaly",dataType="Boolean")
    AnomalyMessage = message(name="AnomalyMessage",featureList=[Anomaly])

    # legitimate message
    Legitimate = data(name="legitimate",dataType="Boolean")
    legitimateMessage = message(name="legitimateMessage",featureList=[Legitimate])

    #-----------------------------------------------------------------------------------------------------------------------
    #--------------------------------------- LOGICAL ARCHITECTURE ----------------------------------------------------------
    #-----------------------------------------------------------------------------------------------------------------------
    adaptiveSystem = system(name="adaptiveSystem", description="Example adaptive system",messageList=[weatherConditions,shipPose,shipAction,predictedPath,AnomalyMessage,legitimateMessage])

    #-A- --- managed system ---
    managedSystem = system(name="managedSystem", description="managed system part")

    weatherConditions_OUT = outport(name="weatherConditions",type="event data", message=weatherConditions)
    shipPose_OUT = outport(name="shipPose",type="event data", message=shipPose)
    shipAction_OUT = outport(name="shipAction",type="event data", message=shipAction)
    predictedPath_IN = inport(name="predictedPath",type="event data", message=predictedPath)

    managedSystem.addFeature(weatherConditions_OUT)
    managedSystem.addFeature(shipPose_OUT)
    managedSystem.addFeature(shipAction_OUT)
    managedSystem.addFeature(predictedPath_IN)

    #-B- --- managing system ---

    managingSystem = system(name="managingSystem", description="managing system part")

    weatherConditions_IN = inport(name="weatherConditions",type="event data", message=weatherConditions)
    shipPose_IN = inport(name="shipPose",type="event data", message=shipPose)
    shipAction_IN = inport(name="shipAction",type="event data", message=shipAction)
    predictedPath_OUT = outport(name="predictedPath",type="event data", message=predictedPath)

    managingSystem.addFeature(weatherConditions_IN)
    managingSystem.addFeature(shipPose_IN)
    managingSystem.addFeature(shipAction_IN)
    managingSystem.addFeature(predictedPath_OUT)

    # connections
    c1 = connection(source=weatherConditions_OUT, destination=weatherConditions_IN)
    c2 = connection(source=shipPose_OUT, destination=shipPose_IN)
    c3 = connection(source=shipPose_OUT, destination=shipPose_IN)
    c4 = connection(source=predictedPath_OUT, destination=predictedPath_IN)


    #---------------------COMPONENT LEVEL---------------------------

    #-MONITOR-
    monitor = process(name="monitor", description="monitor component")

    _weatherConditions = inport(name="weatherConditions",type="event data", message=weatherConditions)
    _shipPose = inport(name="shipPose",type="event data", message=shipPose)
    _shipAction = inport(name="shipAction",type="event data", message=shipAction)
    _pathEstimate = outport(name="pathEstimate",type="event data", message=predictedPath)

    monitor.addFeature(_weatherConditions)
    monitor.addFeature(_shipPose)
    monitor.addFeature(_shipAction)
    monitor.addFeature(_pathEstimate)

    shipPoseEstimation = thread(name="shipPoseEstimation",featureList=[_weatherConditions,_shipPose,_shipAction,_pathEstimate],eventTrigger='newData')
    monitor.addThread(shipPoseEstimation)


    #-ANALYSIS-
    analysis = process(name="analysis", description="analysis component")

    _pathEstimate = inport(name="pathEstimate",type="data", message=predictedPath)
    _pathAnomaly = outport(name="pathAnomaly",type="event data", message=AnomalyMessage)

    analysis.addFeature(_pathEstimate)
    analysis.addFeature(_pathAnomaly)

    analyzePathPredictions = thread(name="analyzePathPredictions",featureList=[_pathEstimate,_pathAnomaly],eventTrigger='anomaly')
    analysis.addThread(analyzePathPredictions)


    #-PLAN-
    plan = process(name="plan", description="plan component")

    #TODO: define input
    _plan = outport(name="plan",type="event data", message=predictedPath)

    plan.addFeature(_plan)

    planner = thread(name="planner",featureList=[_plan])
    plan.addThread(planner)

    #-LEGITIMATE-
    legitimate = process(name="legitimate", description="legitimate component")

    #-EXECUTE-
    execute = process(name="execute", description="execute component")

    _plan = inport(name="plan",type="event data", message=predictedPath)
    _isLegit = inport(name="isLegit",type="event data", message=legitimateMessage)
    _pathEstimate = outport(name="pathEstimate",type="event data", message=predictedPath)

    execute.addFeature(_plan)
    execute.addFeature(_isLegit)
    execute.addFeature(_pathEstimate)

    executer = thread(name="executer",featureList=[_plan,_isLegit,_pathEstimate])
    execute.addThread(executer)

    #-KNOWLEDGE-
    knowledge = process(name="knowledge", description="knowledge component")

    _weatherConditions = port(name="weatherConditions",type="event data", message=weatherConditions)
    _shipPose = port(name="shipPose",type="event data", message=shipPose)
    _shipAction = port(name="shipAction",type="event data", message=shipAction)
    _pathEstimate = port(name="pathEstimate",type="event data", message=predictedPath)
    _pathAnomaly = port(name="pathAnomaly",type="event data", message=AnomalyMessage)
    _plan = port(name="plan",type="event data", message=predictedPath)
    _isLegit = port(name="isLegit",type="event data", message=legitimateMessage)

    knowledge.addFeature(_weatherConditions)
    knowledge.addFeature(_shipPose)
    knowledge.addFeature(_shipAction)
    knowledge.addFeature(_pathEstimate)
    knowledge.addFeature(_pathAnomaly)
    knowledge.addFeature(_plan)
    knowledge.addFeature(_isLegit)

    managingSystem.addProcess(monitor)
    managingSystem.addProcess(analysis)
    managingSystem.addProcess(plan)
    managingSystem.addProcess(legitimate)
    managingSystem.addProcess(execute)
    managingSystem.addProcess(knowledge)

    #---------------------SYSTEM LEVEL---------------------------
    adaptiveSystem.addSystem(managingSystem)
    adaptiveSystem.addSystem(managedSystem)


    #-----------------------------------------------------------------------------------------------------------------------
    #--------------------------------------- PHYSICAL ARCHITECTURE ---------------------------------------------------------
    #-----------------------------------------------------------------------------------------------------------------------

    # XEON PROCESSOR CONNTECTION
    MIPSCapacity = characteristic(name="MIPSCapacity",value=1000.0,dataType="MIPS")
    I1 = port(name="I1",type="event data")
    XeonSolo = processor(name="Xeon",propertyList=[MIPSCapacity],featureList=[I1])


    # XEON PROCESSOR CONNTECTION
    MIPSCapacity = characteristic(name="MIPSCapacity",value=2000.0,dataType="MIPS")
    I2 = port(name="I2",type="event data")
    RPI = processor(name="Raspberry Pi 4B",propertyList=[MIPSCapacity],featureList=[I2])

    # WIFI CONNTECTION
    BandWidthCapacity = characteristic(name="BandWidthCapacity",value=100.0,dataType="Mbytesps")
    Protocol = characteristic(name="Protocol",value="MQTT",dataType="-")
    DataRate = characteristic(name="DataRate",value=100.0,dataType="Mbytesps")
    WriteLatency = characteristic(name="WriteLatency",value=4,dataType="Ms")
    interface = bus(name="interface",propertyList=[BandWidthCapacity,Protocol,DataRate,WriteLatency])

    interface.addConnection(I1)
    interface.addConnection(I2)

    return adaptiveSystem


    #-----------------------------------------------------------------------------------------------------------------------
    #--------------------------------------- MAPPING ARCHITECTURE ----------------------------------------------------------
    #-----------------------------------------------------------------------------------------------------------------------

example1=example()

x=1

