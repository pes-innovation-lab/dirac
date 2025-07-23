Dracon
---

JAEMM -> Job assignment , election and migration mandate 

agent initialization and zone assignment

> on startup, each agent is assigned an agent id and position (pos\_x, pos\_y)
> each agent calculates which zone it belongs to based on its position
> the map is divided into zones as a 3x3 grid by default 

zone leadership election

> agents in the same zone participate in a leader election
> each agent publishes its distance to the center of the zone
> after a set time, the agent closest to the center becomes the zone leader
> if there is a tie, the agent with the lowest agent id is chosen
> the zone leader manages job allocation in the zone

heartbeat and super leader

> each zone leader publishes a heartbeat message
> the agent with agent id == 1 acts as the super leader //for now need to implement a different logic for this selection...
> the super leader monitors heartbeats from all zones
> it receives incoming jobs and routes them to the correct zone based on job coordinates

job publishing and routing

> jobs are published by a separate node called job_publisher.cpp to the /incoming_jobs topic
> the super leader receives these jobs and routes them to the correct zone topic (/zone_x/jobs) based on job location



job bidding and assignment

> the zone leader receives jobs for its zone and starts a bidding process
> it broadcasts the job to all agents in the zone
> each agent that is not busy calculates a bid using manhattan distance from itself to the job location
> each agent sends its bid to the leader
> after a short bidding window, the leader assigns the job to the agent with the lowest bid



job execution and completion

> the assigned agent executes the job (simulated using a timer for now.. this will need to be marked after the path findin is implemented)
> when the job is complete, the agent publishes a completion message
> the agent then becomes available for new jobs



message types

> all communication between agents and for job management uses custom ros2 messages defined in the msg directory
> agentdistance - used for election
> heartbeat - used for leader liveness
> jobassignment - contains job details and status
> jobbid - used for bidding process


workflow  

job publisher -> /incoming_jobs -> super leader -> /zone\_x/jobs -> zone leader
zone leader -> /jobs -> agents in the zone
agents -> /zone_x/job_bids -> zone leader
zone leader -> /jobs (with assignment) -> assigned agent
assigned agent -> /jobs (with completion) -> zone leader and other agents




---

how to run the code 
clone the repo and make a workspace and add the src directory to the workspace
1.  **Source your ROS 2 environment:**
    `source /opt/ros/humble/setup.bash`
2.  **Build the `agentswarm` package:**
    `colcon build --packages-select agentswarm`
3.  **Source your workspace's install setup:**
    `source install/setup.bash`
4.  **Launch the `spawnagent` node:**
    `ros2 launch agentswarm spawnagent.launch.py`
