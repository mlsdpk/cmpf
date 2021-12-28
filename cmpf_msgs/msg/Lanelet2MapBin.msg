# https://github.com/Autoware-AI/messages/blob/master/autoware_lanelet2_msgs/msg/MapBin.msg
# This contains the lanelet map in binary format.

# Timestamp and frame_id of the map
Header header

# version of map format. keep this as empty string if unnecessary
string format_version

# version of map version. keep this as empty string if unnecessary
string map_version

# binary data of lanelet2 map
int8[] data
