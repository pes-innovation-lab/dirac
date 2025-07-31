# Big Tank Multi-Agent Pathfinding System

A distributed multi-agent pathfinding system using the Big Tank algorithm with gossip protocol for state synchronization.

## Overview

This system implements:
- **Big Tank Algorithm**: Force-based pathfinding with discrete movement
- **Gossip Protocol**: Distributed state dissemination with consensus mechanism
- **Tick-based Synchronization**: Global tick counter with leader-follower coordination
- **A* Integration**: Optimal path planning with force-based movement
- **ROS2 Communication**: Robust distributed agent communication

## Instruction to run

### 1. Make the Script Executable (First Time Only)

```bash
chmod +x run_agents.sh
```

### 2. Run the System

```bash
./run_agents.sh
```

This script will:
- Build the workspace (`colcon build --packages-select bt`)
- Source the environment (`source install/setup.bash`)
- Launch both agents with slowed-down processing (2-second delays for observation)

### 3. Observe the System

The agents will start processing ticks with a 2-second delay between each tick, making it easy to observe:

- **Agent Movement**: Watch agents move along their A* paths using BigTank forces
- **State Updates**: Observe distributed state synchronization
- **Tick Progression**: See the leader advance global ticks after all agents acknowledge

### 4. Monitor ROS2 Topics

In separate terminals, you can monitor the system's communication:

#### Watch State Updates
```bash
# Monitor agent state updates (CSV format)
ros2 topic echo /db_update
```

#### Watch Acknowledgments
```bash
# Monitor consensus acknowledgments
ros2 topic echo /state_ack
```

#### Watch Global Tick Progression
```bash
# Monitor leader's tick advancement
ros2 topic echo /global_tick
```

#### List All Topics
```bash
# See all active topics
ros2 topic list
```

## System Configuration

### Agent Configuration
Agents are configured in `agents.csv`:
```csv
agent_id,start_x,start_y,priority,job_id,goal_x,goal_y,is_leader
1,2,3,1,J1,2,1,true
2,1,2,2,J2,3,2,false
```

### Map Configuration
Map layout is defined in `map.csv`:
```csv
0,0,0,0
0,1,1,0
0,1,1,0
0,0,0,0
```
Where:
- `0` = free space
- `1` = obstacle

## Architecture

### Components

1. **Agent**: Main agent class with BigTank pathfinding integration
2. **StateUpdater**: Handles gossip protocol and distributed coordination
3. **BigTank**: Force-based pathfinding algorithm implementation
4. **AgentStateDB**: Agent state management and persistence
5. **PathPlanner**: A* pathfinding for optimal route calculation

### Communication Flow

1. **State Publishing**: Each agent publishes its state as CSV on `/db_update`
2. **State Reception**: Agents receive and parse other agents' states
3. **Acknowledgment**: Agents send ACK when they have all states for a tick
4. **Tick Advancement**: Leader advances global tick when all agents acknowledge
5. **Periodic Republishing**: 500ms timer ensures robust state exchange

### Consensus Mechanism

The system uses a gossip protocol with consensus:
- Agents must receive all other agents' states before acknowledging
- Leader only advances tick when ALL agents have acknowledged
- Periodic republishing prevents missed messages
- Self-tick processing ensures leader participates in its own coordination


## Development

### Building
```bash
cd dirac/bt_ws
colcon build --packages-select bt
```

### Running Individual Components
```bash
# Source environment
source install/setup.bash

# Run single agent
ros2 run bt agent --ros-args -p agent_id:=1

# Use launch file for multiple agents
ros2 launch bt simple_agents.launch.py
```

### Debugging
- Agents log detailed information about movement, forces, and coordination
- Use `RCLCPP_DEBUG` logging level for more verbose output
- Monitor ROS2 topics to observe real-time communication

## Stopping the System

Press `Ctrl+C` in the terminal running `run_agents.sh` to cleanly stop all agents.

---

**Note**: The system currently runs with 2-second delays between ticks for easy observation. Remove the `std::this_thread::sleep_for(std::chrono::seconds(2));` line in `agent.cpp` for full-speed operation.
