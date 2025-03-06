# Robot Anomaly Detection and Compensation System

This project is designed to run a distributed MAPE-K loop for anomaly detection and compensation in a simulated Turtlebot4 environment. The simulation includes handling lidar occlusion and re-plan the robot's navigation tool dynamically.

## Prerequisites

Before running the system, ensure the following requirements are fulfilled:

1. **Python Dependencies**: Install necessary Python libraries by running:
   ```bash
   pip install robosapiensio
   ```

2. **MQTT Broker**: An MQTT broker (such as Mosquitto) must be running on your PC.

3. **Redis Database**: A Redis server must be active to store and manage data shared among MAPE-K components.

## Running the Simulation

Follow these steps to run the simulation:

1. **Start the MQTT Broker and Redis Server**:
   - Ensure your MQTT broker is running.
   - Start your Redis database server.

2. **Run the Robot Simulator**:
   - Execute `simulator.py` to start the robot simulation:
     ```bash
     python3 Realization/ManagingSystem/Simulator/Turtlebotsim.py
     ```
   - This will open a dashboard where you can control the robot and simulate lidar occlusion.

3. **Initialize MAPE-K Components**:
   - Run `main.py` to initialize all MAPE-K components:
     ```bash
     python3  Resources/main.py
     ```
   - `main.py` will set up the MAPE-K loop, subscribing to necessary topics and managing anomaly detection.
## MAPE-K Component Development

You can also see how the MAPE-K components are developed by checking the `Realization/ManagingSystem/Nodes` directory. There is a separate `.py` file for each of the components, detailing their implementation and role in the system.
## System Behavior

Once the entire system is running:
- When you press the **Lidar Occlusion** button in the simulator, the MAPE-K loop will detect the occlusion anomaly.
- The system will execute a new plan to re-plan the robot navigation and compensate for the occlusion. You should see the robot navigation method is changed.

## Notes
- Make sure your MQTT broker and Redis server are correctly configured and running before starting the simulation.
- Feel free to explore and modify the code to customize the MAPE-K loop behavior for different scenarios.
- You can also see log file of the MAPE-K components by checking the `Realization/Resources/MAPE_tets.log` directory. Detailing the functionality of the components. 

