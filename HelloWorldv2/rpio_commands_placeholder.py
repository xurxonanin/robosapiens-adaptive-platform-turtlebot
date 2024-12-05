from rpio.transformations.transformations import swc2code_py,message2code_py, swc2launch, swc2main,swc2dockerCompose,update_robosapiensIO_ini,add_backbone_config
from HelloWorldv2.Design.AADLIL import *
from rpio.utils.auxiliary import *
from rpio.pyLauncher.pyLauncher import launch,launch_main

#-------------------------------------------------------------------------------------------------------
# !! THESE COMMANDS NORMALLY ARE CALLED FROM THE RPIO CLI, FOR TESTING PURPOSE, A PYTHON FILE IS USED !!
#-------------------------------------------------------------------------------------------------------
# 0. GENERATE ROBOSAPIENSIO CODE PACKAGE
packageName = "HelloWorldv2"

# 1. GENERATE AADL FROM ROBOARCH (NOT IMPLEMENTED YET)

# 2. GENERATE AADL INTERMEDIATE LANGUAGE FROM AADL (NOT IMPLEMENTED YET)

# 2. LOAD THE AADL INTERMEDUATE LANGUAGE (MOCKUP)
design = HelloWorld_v2()


# 3. GENERATE CUSTOM MESSAGES FROM AADL INTERMEDIATE LANGUAGE
try:
    message2code_py(system=design, path="Realization/ManagingSystem/Messages")
    message2code_py(system=design, path="Realization/ManagedSystem/Messages")
except:
    print("Failed to generate the messages")

# 4. GENERATE SWC CODE FROM AADL INTERMEDIATE LANGUAGE
try:
    swc2code_py(system=design,path="Realization/ManagingSystem/Nodes")
except:
    print("Failed to generate the software components")

# 5. GENERATE SWC LAUNCH FILES FOR THE IDENTIFIED PROCESSOR BINDINGS
try:
    swc2launch(system=design.systems[0],path="Realization/ManagingSystem/Platform")
    swc2launch(system=design.systems[1], path="Realization/ManagedSystem/Platform")
except:
    print("Failed to generate the software component launch files")

# 6. GENERATE SWC MAIN FILE FOR THE IDENTIFIED PROCESSOR BINDINGS
try:
    swc2main(system=design.systems[0],package=packageName,prefix=None,path="Resources")
except:
    print("Failed to generate the software component main file for the given platforms")

# 7. GENERATE DOCKER COMPOSE FILE FOR THE IDENTIFIED PROCESSOR BINDINGS
try:
    swc2dockerCompose(system=design.systems[0],path="Realization/ManagingSystem/Platform")
    add_backbone_config(system=design,path="Resources")
except:
    print("Failed to generate the docker compose for the given platforms")

# 8. update the roboSapiensIO.ini file based on the generation
try:
    update_robosapiensIO_ini(system=design,path=None)
except:
    print("Failed to update the robosapiensIO.ini file.")