# Synchronization record published once per training frame.
#
# header.stamp is the wall-clock time when this sync frame was taken.
# The MCAP-to-LeRobot converter uses this as the canonical frame timestamp.
#
# Each topic_stamps entry records which message (identified by its header stamp)
# was captured for a given topic at this sync point.  The converter uses those
# stamps to look up the matching messages and assemble one training sample per
# RecordData message.
std_msgs/Header header
isaac_ros_data_flywheel/TopicStamp[] topic_stamps
