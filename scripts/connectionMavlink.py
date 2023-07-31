from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection(device='udp:192.168.2.1:14550', baudrate=115200)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
print("wait_heartbeat...")
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
print("Success...")

# Once connected, use 'the_connection' to get and send messagesñ