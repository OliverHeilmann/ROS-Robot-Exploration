# File: rosbot_msgs/msg/Match.msg

int32 class_id
float32 confidence
int32[4] box  # Assuming the box is defined by 4 integers (x, y, width, height)
float32[4] color  # Assuming color is defined by 4 floats (RGBA)