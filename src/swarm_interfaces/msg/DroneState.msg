uint8  drone_id        # Swarm ID: 1, 2, 3
float32 x              # Local NED position X (north, metres)
float32 y              # Local NED position Y (east, metres)
float32 z              # Local NED position Z (down, metres; negative = up)
float32 vx             # Velocity X (m/s)
float32 vy             # Velocity Y (m/s)
float32 vz             # Velocity Z (m/s)
uint8  arming_state    # 1=disarmed 2=armed
float32 altitude
uint8  swarm_state     # 0=idle 1=takeoff 2=hold 3=formation 4=return 5=land 6=done
uint32[] completed_task_ids # Array of task IDs finished by this drone
builtin_interfaces/Time stamp  # heartbeat — receiver marks peer failed if silent > 1.5s
