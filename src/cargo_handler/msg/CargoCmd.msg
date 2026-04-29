# CargoCmd.msg
# Command sent to the cargo handler via ROS to trigger pick-up or delivery.

# Available actions
string ACTION_PICKUP  = "pickup"
string ACTION_DELIVER = "deliver"
string ACTION_ABORT   = "abort"

string action          # "pickup" | "deliver" | "abort"
string cargo_id        # identifier of the cargo (e.g. "cargo_1")
geometry_msgs/Point target_position   # delivery target (world frame), used when action=="deliver"
