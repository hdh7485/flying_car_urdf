#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('virtual_grid', OccupancyGrid, queue_size=1)
    rospy.init_node('grid_publisher', anonymous=True)
    rate = rospy.Rate(1)
    header = Header()
    header.frame_id = 'map'
    seq = 0
    resolution = 1.0
    width = 6.0
    height = 6.0
    origin_pose = Pose()
    origin_pose.position.x = 0.0
    origin_pose.position.y = 0.0
    # origin_pose.position.x = -width/2
    # origin_pose.position.y = -height/2
    origin_pose.position.z = 0.0
    origin_pose.orientation.x = 0.0
    origin_pose.orientation.y = 0.0
    origin_pose.orientation.z = 0.0
    origin_pose.orientation.w = 1.0
    while not rospy.is_shutdown():
        grid = OccupancyGrid()
        header.seq = seq
        header.stamp = rospy.Time.now()
        grid.header = header
        grid.info.map_load_time = rospy.Time.now()
        # The map resolution [m/cell]
        grid.info.resolution = resolution
        # Map width [cells]
        grid.info.width = width
        # Map height [cells]
        grid.info.height = height
        # The origin of the map [m, m, rad].  
        # This is the real-world pose of the cell (0,0) in the map.
        grid.info.origin = origin_pose

        # The map data, in row-major order, starting with (0,0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.
        grid.data = [0, 0, 0, 0, 0, 0,
                     0, 100, 0, 0, 0, 0,
                     0, 0, 100, 100, 0, 0,
                     0, 0, 100, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0]

        pub.publish(grid)
        rate.sleep()
        seq += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass