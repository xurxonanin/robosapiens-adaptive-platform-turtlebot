from NTNU.Realization.ManagingSystem.Nodes.monitor.monitor import Monitor
from NTNU.Realization.ManagingSystem.Nodes.analysis.analysis import Analysis
from NTNU.Realization.ManagingSystem.Nodes.plan.plan import Plan
from NTNU.Realization.ManagingSystem.Nodes.execute.execute import Execute
import time

_Monitor = Monitor("../Realization/ManagingSystem/Nodes/monitor/config.yaml")
_Analysis = Analysis("../Realization/ManagingSystem/Nodes/analysis/config.yaml")
_Plan = Plan("../Realization/ManagingSystem/Nodes/plan/config.yaml")
_Execute = Execute("../Realization/ManagingSystem/nodes/Execute/config.yaml")

_Monitor.register_callbacks()
_Analysis.register_callbacks()
_Plan.register_callbacks()
_Execute.register_callbacks()

_Monitor.start()
_Analysis.start()
_Plan.start()
_Execute.start()

try:
    print("Script is running. Press Ctrl+C to stop.")
    while True:
        time.sleep(1)  # Sleep to avoid busy-waiting
except KeyboardInterrupt:
    _Monitor.shutdown()
    _Analysis.shutdown()
    _Plan.shutdown()
    _Execute.shutdown()
    print("\nKeyboard interruption detected. Exiting...")