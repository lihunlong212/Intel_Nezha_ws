# CargoDetection.msg
# Result of a single visual-recognition frame.

std_msgs/Header header
bool detected                      # true when cargo is visible in frame
string cargo_id                    # which cargo was detected
float32 center_x                   # pixel column of cargo centre
float32 center_y                   # pixel row of cargo centre
float32 area                       # contour area in pixels²
float32 offset_x                   # horizontal offset from frame centre (pixels)
float32 offset_y                   # vertical offset from frame centre (pixels)
float32 distance                   # estimated distance to cargo (metres, 0 if unavailable)
