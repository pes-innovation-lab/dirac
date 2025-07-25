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
