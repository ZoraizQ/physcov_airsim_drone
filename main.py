import airsim
import threading
import sys
import os
import math
import time
import argparse
import pprint
from airsim.types import DrivetrainType, Vector3r
import numpy as np
from numpy.lib.function_base import angle
from tensorboard.compat.tensorflow_stub.tensor_shape import vector 
from rsr import get_rsr_signature
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle
# import open3d as o3d


# def visualize_pointcloud(points_3d): 
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points_3d)
#     o3d.visualization.draw_geometries([pcd])


def angle_between(v1, v2): # angle between 2 unit vector
    return np.degrees(np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0)))


class PhysCovNavigator(threading.Thread):
    def __init__(self, args):
        self.boxsize = args.size
        self.stripewidth = args.stripewidth
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.result = None
        self.trip_time = 0
        self.recording = False
        self.path  = []


        self.runThread = False
        
        threading.Thread.__init__(self)

    def run(self): 
        print ("Starting thread")
        self.get_lidar_data()
        print ("Exiting thread")



    def execute(self):
        print("arming the drone...")
        self.client.armDisarm(True)


        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print("getMultirotorState: %s" % s)

        if not self.recording:
            landed = self.client.getMultirotorState().landed_state
            if landed == airsim.LandedState.Landed:
                print("taking off...")
                self.client.takeoffAsync().join()

            landed = self.client.getMultirotorState().landed_state
            if landed == airsim.LandedState.Landed:
                print("takeoff failed - check Unreal message log for details")
                return
        '''
        # AirSim uses NED coordinates so negative axis is up.
        x = -self.boxsize
        z = -self.altitude

        print("climbing to altitude: " + str(self.altitude))
        self.client.moveToPositionAsync(0, 0, z, self.velocity).join()

        print("flying to first corner of survey box")
        self.client.moveToPositionAsync(x, -self.boxsize, z, self.velocity).join()
        
        # let it settle there a bit.
        self.client.hoverAsync().join()
        time.sleep(2) 

        # after hovering we need to re-enabled api control for next leg of the trip
        self.client.enableApiControl(True)

        # now compute the survey path required to fill the box 
        path = []
        distance = 0
        while x < self.boxsize:
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            x += self.stripewidth            
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, -self.boxsize, z)) 
            x += self.stripewidth  
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, -self.boxsize, z))
            distance += self.boxsize 
        
        
        # if self.runThread:
        #     self.start()

        print("starting survey, estimated distance is " + str(distance))
        '''

        if not self.recording:
            self.path = np.load('recordedpath.npy').tolist()
            self.path = [Vector3r(-i[0], -i[1], -i[2]) for i in self.path]
            print(self.path)
            self.trip_time = len(self.path)
        else:
            self.trip_time = 500 #distance // self.velocity
        print("estimated survey time is " + str(self.trip_time))

        
        try:
            self.start()
            if not self.recording:
                self.result = self.client.moveOnPathAsync(self.path, self.velocity, self.trip_time, 
                airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), self.velocity + (self.velocity/2), 1) #removed join
            #     self.result = self.client.moveOnPathAsync(self.path, self.velocity, self.trip_time, self.velocity + (self.velocity/2), 1) #removed join
            
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass

        

        self.join()
        

        print('finished getting lidar readings')



    def get_lidar_data(self):
        N = 0 # number of unique RSR signatures
        x = 5 # number of RSR beams / points
        drone_center_list = []
        
        reach_range = 6
        g = 2 # granularity (beam collision detection step intervals)
        radius = 1
        physcov = 0
        denom = pow(reach_range//g, x)  # big number otherwise, so for now x/10
        unique_rsr_signatures = []
        # timesteps = self.trip_time
        timesteps = 600
        collision_angle_thresh = 20 # degrees

        physcovs = []
        # gap = 2147483647-denom
        unique_coll_vectors = []
        unique_coll_timesteps = []

        print(f'denom = {denom}')

        print('Path:', len(self.path))
        collision_happened_before = False
        start = time.time()
        p = 0
        for i in range(0, timesteps): #change to timesteps
            # you have to edit Documents/AirSim/settings.json as follows https://microsoft.github.io/AirSim/lidar/
            lidarData = self.client.getLidarData(lidar_name="LidarSensor1", vehicle_name= "Drone1")

            physcov = N * 100 / denom
            physcovs.append(physcov)

            collision_happened = False

            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                # print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                # print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                # print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
                
                # calling the func with the 3d points array ([(x,y,z)...])
                print('Time',i)
                drone_center = lidarData.pose.position.to_numpy_array() * -1
                drone_center_list.append(drone_center)  
                
                new_rsr = get_rsr_signature(lidar_points_orig=points, DRONE_CENTER=drone_center, NUM_PTS=x, 
                                            GRANULARITY=g, REACH_RANGE=reach_range, RADIUS=radius, plot_lidar=False)

                is_unique = True
                for signature in unique_rsr_signatures:
                    if np.array_equal(new_rsr, signature):
                        is_unique = False

                if is_unique: # is unique
                    unique_rsr_signatures.append(new_rsr)
                    N += 1    
                    # print(N, g_x)
                    print("Updated Physcov:","{:e}".format(physcov),"%")
                    print(new_rsr)

                # CHECK COLLISIONS
                collision_info = self.client.simGetCollisionInfo()

                if collision_info.has_collided:
                    collision_happened = True
                    impact_point = collision_info.impact_point.to_numpy_array() * -1
                    drone_pos = collision_info.position.to_numpy_array() * -1
                    
                    new_coll_vector = impact_point - drone_pos
                    new_coll_vector /= np.linalg.norm(new_coll_vector) # converting to unit vector

                    if len(unique_coll_vectors) == 0:
                        unique_coll_vectors.append(new_coll_vector)
                        unique_coll_timesteps.append(i)
                        print('UNIQUE collision')
                    else: # check if unique
                        is_unique = True
                        for prev_coll_vector in unique_coll_vectors:
                            angle_between_prev =  angle_between(new_coll_vector, prev_coll_vector) 
                            # print('angle:', angle_between_prev)
                            if angle_between_prev < collision_angle_thresh: # then they are too close, not unique
                                is_unique = False
                                break
                        
                        if is_unique:
                            unique_coll_vectors.append(new_coll_vector)
                            unique_coll_timesteps.append(i)
                            print('UNIQUE collision')
            
            

            if not collision_happened: # since unique collision checks already take some time
                time.sleep(0.2)

            if collision_happened:
                collision_happened_before = True
            
            p += 1
            # time.sleep(5)

            # self.write_lidarData_to_disk(points, i)
        elapsed_time = (time.time() - start)

        print('========\nSIMULATION COMPLETE\n========')
        print(f'Timesteps: {timesteps}')
        print(f'Elapsed time: {elapsed_time} s')
        print(f'Average timestep: {elapsed_time / timesteps} s')


        ax = plt.gca()

        physcov_data = {'physcovs' : physcovs, 'g' : g, 'reach_range': reach_range, 'x': x, 'N': N, 'timesteps' : timesteps, 'radius': radius, 'unique_rsr_signatures': unique_rsr_signatures, 'unique_coll_vectors': unique_coll_vectors, 'unique_coll_timesteps': unique_coll_timesteps}
        
        file_params_string = 'g-'+str(g)+'_x-'+str(x)+'_N-'+str(N)+'_reach-'+str(reach_range)+'_rad-'+str(radius)

        plt.plot(np.arange(timesteps), physcovs, label = 'RSR-'+str(x))
        plt.xlabel('timesteps')
        plt.ylabel('physcov %')
        
        # ax.text(x = 0.97, y = 0.68, s  = '100', fontsize = 12)
        plt.text(x = 0.97, y = 0.95, s = ('g: '+str(g)), fontsize=15, ha='right', fontweight='book', transform=ax.transAxes)
        plt.text(x = 0.97, y = 0.88, s  = ('N: '+str(N)), fontsize=15, ha='right', fontweight='book', transform=ax.transAxes)
        
        plt.text(x = 0.97, y = 0.81, s = 'reach_range: '+str(reach_range), fontsize=15, ha='right', fontweight='book', transform=ax.transAxes)
        plt.text(x  = 0.97, y = 0.74, s = 'radius: '+str(radius), fontsize=15, ha='right', fontweight='book', transform=ax.transAxes)
        plt.legend()
        plt.savefig('physcov_'+file_params_string+'.png')
        plt.show()

        if len(unique_coll_timesteps) != 0:
            plt.clf()
            plt.plot(unique_coll_timesteps, np.arange(1, len(unique_coll_timesteps)+1), 'r')
            plt.xlabel('timesteps')
            plt.ylabel('# unique collisions')
            plt.xlim([0, timesteps]) # 0-100
            plt.savefig('num_unique_coll_'+file_params_string+'.png')
            plt.show()

            U, V, W = zip(*unique_coll_vectors)
            XYZ = np.zeros(len(U)).tolist()

            plt.clf()

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.quiver(XYZ, XYZ, XYZ, U, V, W)
            ax.set_xlim([-1,1])
            ax.set_ylim([-1,1])
            ax.set_zlim([-1,1])
            plt.title('Unique Collision Vectors Set')
            plt.savefig('uniquecoll_'+file_params_string+'.png')
            plt.show()
        else:
            print('No collisions occurred.')

        with open('physcov_'+file_params_string+'.pkl', 'wb') as f:
            pickle.dump(physcov_data, f)
    
        drone_center_list = np.array(drone_center_list)
        if (self.recording):
            np.save('recordedpath.npy', drone_center_list)

        print("Final Physcov:","{:e}".format(physcov),"%")



    def parse_lidarData(self, data):
        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        # print(points[:500])
        points = np.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def write_lidarData_to_disk(self, points, i):
        save_path = os.path.join(os.curdir, 'LidarOutputs', 'lidar_t'+str(i)+'.npy')
        print(save_path)

        with open(save_path, 'wb') as f: # save points for all these timestamps
            np.save(f, points)


    def stop(self):

        airsim.wait_key('Press any key to reset to original state')


        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")


if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Usage: survey boxsize stripewidth altitude")
    arg_parser.add_argument("--size", type=float, help="size of the box to survey", default=50)
    arg_parser.add_argument("--stripewidth", type=float, help="stripe width of survey (in meters)", default=10)
    arg_parser.add_argument("--altitude", type=float, help="altitude of survey (in positive meters)", default=25)
    arg_parser.add_argument("--speed", type=float, help="speed of survey (in meters/second)", default=6)
    args = arg_parser.parse_args(args)

    nav = PhysCovNavigator(args)
    try:
        nav.execute()
    finally:
        nav.stop()
