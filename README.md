# UAV_Obstacle_Avoidance_Vortex_Field
##🚁 **Reactive UAV Person-Following with Vortex-Based Obstacle Avoidance**

This project presents the design and validation of a reactive control architecture for a quadrotor UAV performing person-following in cluttered environments.

The UAV does not navigate toward a static goal. Instead, it maintains a desired relative pose with respect to a moving target (Bill), following it at a fixed distance and altitude offset. Obstacle avoidance is handled independently from tracking through a combination of repulsive fields and vortex (tangential) fields, without introducing any classical attractive potential toward a goal.

🧭 Control Philosophy

The system is fully reactive and does not rely on global path planning for the UAV. The drone continuously:

• Computes a virtual setpoint behind the target
• Maintains a desired following distance
• Reacts to obstacles detected through onboard proximity sensors

Obstacle avoidance is based on two complementary components:

🔴 Repulsive Field
Pushes the UAV away from obstacles when entering their influence region.
This guarantees collision avoidance but alone may cause oscillations or local minima.

🌀 Vortex (Tangential) Field
Adds a rotational component around obstacles.
Instead of pushing the drone straight back, it generates a lateral sliding motion that guides the UAV smoothly around obstacles.

The final obstacle-induced action is:

F_obs = F_rep + F_vor


This ensures both safety (repulsion) and smooth circumnavigation (vortex).

⚙️ Stabilization and Blending Mechanisms

To avoid instability and aggressive corrections, several heuristic stabilizations are implemented:

• Smooth blending between tracking and avoidance
• Cubic shaping of avoidance activation
• Low-pass filtering of avoidance forces
• Saturation of lateral forces
• Hysteresis logic to prevent chattering

These mechanisms ensure that obstacle avoidance acts as a bounded perturbation, rather than dominating the tracking objective.

🔬 Simulation Scenarios

The repository includes four different simulation scenarios, progressively increasing in complexity:

🟢 Scenario 1 – No Obstacles
Baseline person-following behavior.
The UAV maintains the desired relative pose without interference.

🟡 Scenario 2 – Obstacles without Heuristics (Unstable Behavior)
Repulsive + vortex fields active but without smoothing/blending.
The system shows instability, sensitivity to perturbations, and non-repeatable trajectories.

🟠 Scenario 3 – Obstacles with Heuristic Stabilization (Stable Behavior)
Full avoidance strategy with smoothing and blending enabled.
The UAV smoothly bypasses obstacles and consistently re-aligns behind the target.

🔵 Scenario 4 – Corridor Scenario (Dense Obstacles)
Narrow passage with continuous obstacle interaction.
The UAV performs micro-corrections, maintains bounded tracking error, and safely exits the corridor.


📄 Documentation

The repository includes:

• Complete technical report
• Simulation files
• Parameter configurations for all scenarios
• Presentation material

This project demonstrates that combining repulsive and vortex fields, together with proper stabilization heuristics, enables robust real-time UAV person-following in complex environments without requiring global path planning.
