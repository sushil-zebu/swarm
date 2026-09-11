uint8 drone_id                  # Swarm drone ID (1..N)
uint8 swarm_state               # Current state (0=IDLE, 1=TAKEOFF, 2=HOLD, 3=FORMATION, 4=RTL, 5=LAND, 6=DONE, 7=MISSION_EXEC)
uint32 current_waypoint_idx     # Current target waypoint index (0-indexed)
uint32 total_waypoints          # Total count of mission waypoints
float32 distance_to_target      # Euclidean distance to current target (metres)
float32 progress_pct            # Overall mission completion percentage (0.0 .. 100.0)
string status_text              # Human-readable state summary message
builtin_interfaces/Time stamp
