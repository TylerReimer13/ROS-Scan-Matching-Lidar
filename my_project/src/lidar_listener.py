import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from read_scan import *
from matplotlib import pyplot as plt


def transform_pc(pc, rotation, translation):
        new_pts = []

        for pt in pc:
            # Apply rotation and translation to each point in point cloud
            new_pt = np.dot(rotation, pt) + translation
            new_pts.append(new_pt)

        return np.array(new_pts)


class Subscriber:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("scan", LaserScan, self.callback)
        
        self.ang_min = -3.1241390705108643
        self.ang_max = 3.1415927410125732
        self.ang_inc = 0.01745329238474369
        self.rng_min = 0.
        self.rng_max = 8.
	
        self.reader = ScanReader(self.ang_min, self.ang_max, self.ang_inc, self.rng_min, self.rng_max)
        self.icp = ICP()
        
        self.occ_grid = OccupancyGrid(100, 100, [-5., 5.], [-5., 5.])
        
        self.prev_scan = []
        self.curr_scan = []
        
        self.sleep_time = 1.
        self.start_time = rospy.get_rostime().secs
    
        self.ctr = 0
        
        self.pos = np.zeros(2)
        self.heading = 0.
        
        
    def callback(self, msg):
        ranges = msg.ranges
        self.curr_scan = self.reader.convert_rng_to_xy(ranges)


    def __call__(self):
        my_ctr = 0 
        while not rospy.is_shutdown():
             curr_time = rospy.get_rostime().secs
             
             if (curr_time - self.start_time) >= 3.:
                  # print("Subscriber Running")
             
                  if self.ctr == 0:
                       self.prev_scan = self.curr_scan
             
                  if self.ctr >= 2:
                       T, transformed = self.icp(self.curr_scan, self.prev_scan, n_iter=50)
                       translation = -T[:2, 2]
                       rotation = np.sign(T[0, 1]) * acos(T[0, 0])
                       
                       self.pos += translation
                       self.heading += rotation
                       
                       print("Translation: ", translation)
                       print("Rotation: ", rotation)
                       print("Current Position: ", self.pos)
                       print("Current Heading: ", self.heading)
                       print("----------------------------------------------")
                       
                       self.prev_scan = self.curr_scan
                       self.ctr = 0
                       
                       
                  self.ctr += 1
                  my_ctr += 1
                  rospy.sleep(self.sleep_time)
                 
             
        rospy.spin()

if __name__ == '__main__':
    subscriber = Subscriber()
    subscriber()
