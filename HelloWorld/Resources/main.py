from Realization.ManagingSystem.Nodes.Monitor.Monitor import Monitor
from Realization.ManagingSystem.Nodes.Analysis.Analysis import Analysis
from Realization.ManagingSystem.Nodes.Plan.Plan import Plan
from Realization.ManagingSystem.Nodes.Execute.Execute import Execute
import time 


monitor = Monitor("../Realization/ManagingSystem/Nodes/Monitor/config.yaml")
analyse = Analysis("../Realization/ManagingSystem/Nodes/Analysis/config.yaml")
plan = Plan("../Realization/ManagingSystem/Nodes/Plan/config.yaml")
execute = Execute("../Realization/ManagingSystem/Nodes/Execute/config.yaml")

monitor.register_callbacks()
analyse.register_callbacks()
plan.register_callbacks()
execute.register_callbacks()

monitor.start()
analyse.start()
plan.start()
execute.start()

try:
    print("Script is running. Press Ctrl+C to stop.")
    while True:
        time.sleep(1)  # Sleep to avoid busy-waiting
except KeyboardInterrupt:
    monitor.shutdown()
    analyse.shutdown()
    plan.shutdown()
    execute.shutdown()
    print("\nKeyboard interruption detected. Exiting...")