# Python client example to get Lidar data from a drone
#
import airsim

import sys
import os
import math
import time
import argparse
import pprint
import numpy as np
import open3d as o3d 

def visualize_pointcloud(points_3d): 
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    o3d.visualization.draw_geometries([pcd])

# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def execute(self):

        print("arming the drone...")
        self.client.armDisarm(True)

        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print("state: %s" % s)

        # airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()
        #print("state: %s" % pprint.pformat(state))

        # airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        self.client.moveToPositionAsync(10, 50, 10, 25).join()

        self.client.hoverAsync().join()

        # airsim.wait_key('Press any key to get Lidar readings')
        
        for i in range(1, 5):
            # you have to edit Documents/AirSim/settings.json as follows https://microsoft.github.io/AirSim/lidar/
            lidarData = self.client.getLidarData(lidar_name="LidarSensor1", vehicle_name= "Drone1")

            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
                
                # calling the func with the 3d points array ([(x,y,z)...])
                print(points.shape)
                if i == 3:
                    print('Visualizing pointcloud')
                    visualize_pointcloud(points)
                path = os.path.join(os.curdir, 'LidarOutputs', 'lidar_t'+str(i)+'.npy')
                print(path)

                with open(os.path.join(os.curdir, 'LidarOutputs', 'lidar_t'+str(i)+'.npy'), 'wb') as f: # save points for all these timestamps
                    np.save(f, points)

            time.sleep(5)

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        print(points[:500])

        points = np.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def write_lidarData_to_disk(self, points):
        # TODO
        print("not yet implemented")

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()