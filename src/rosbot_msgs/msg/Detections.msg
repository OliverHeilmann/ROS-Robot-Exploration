# File: rosbot/msg/Detections.msg

sensor_msgs/Image image
int32 class_id
float32 confidence
int32[4] box  # the box is defined by 4 integers (x, y, width, height)
float32[4] color  # color is defined by 4 floats (RGBA)
