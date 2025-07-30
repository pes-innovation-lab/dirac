dirac_lib has the code :
## Leader Election Process

**1. Initial Calculation:**
* At startup, each agent calculates its `distance_to_center` for its assigned zone.

**2. Communication Phase:**
* **Publishing:** All agents repeatedly publish their `distance_to_center`, `agent_id`, and `zone_id` on a shared election topic for their zone (e.g., `/zone_1/election`).
* **Subscribing:** Each agent subscribes to this same topic to collect `distance_to_center` values from all other agents in its zone.

**3. Election Logic (after a fixed period, e.g., 15 seconds):**
* Each agent performs the following:
    * Includes its *own* `distance_to_center` in the set of received distances.
    * Identifies the agent with the *smallest* `distance_to_center`.
    * **Tie-breaker:** If multiple agents have the same smallest `distance_to_center`, the agent with the *lowest* `agent_id` is chosen.
    * The chosen agent is elected as the leader for that zone.

**4. Post-Election State:**
* Each agent updates its internal state:
    * `isLeader` is set to `true` if it is the elected leader, otherwise `false`.
    * `z_leader` is set to the `agent_id` of the elected leader.
* **Communication Halt:** Publishing on the election topic stops.
* **Logging:** The elected leader is announced in the logs (e.g., "I am the LEADER for zone X!").
---
# JOB ASSIGNMENT 
## SuperLeader

* **Listens for Jobs:** Subscribes to `/incoming_jobs` for new job messages.
* **Zone Determination:** Determines job's zone based on coordinates (e.g., `determine_zone(x, y)`).
* **Job Routing:** Publishes job to the appropriate zone's topic (e.g., `/zone_1/incoming_jobs`).

## ZoneLeader

* **Receives Job:** Subscribes to its zone's incoming jobs topic (e.g., `/zone_1/incoming_jobs`).
* **Broadcast to Agents:** Publishes the job to all agents in the zone (e.g., `/zone_1/jobs`) and starts a bid collection timer.
* **Collects Bids:** Collects all bids from the zone's bid topic (e.g., `/zone_1/bids`) for a short period (e.g., 2 seconds).
* **Best Bid Selection:** Selects the agent with the lowest bid after the timer expires.
* **Job Assignment:** Assigns the job to the winning agent by publishing the assignment.

## Agent

* **Calculates and Submits Bids:** Receives the job from its zone's job topic, calculates bid cost (e.g., Manhattan distance), and publishes the bid to the zone's bid topic.
