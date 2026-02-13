from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase

from . util import rotateQuaternion, getHeading

from . sensor_model import SensorModel

import random
import math

class PFLocaliser(PFLocaliserBase):
       
    def __init__(self, logger, clock):
        # ----- Call the superclass constructor
        super().__init__(logger, clock)

        self.sensor_model = SensorModel(logger)
        self.logger = logger
        self.weights = []
        
        # ----- Set motion model parameters
        # TO FINE TUNE
        self.ODOM_ROTATION_NOISE = 0.2 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.2 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.2 # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 200     # Number of readings to predict

        # ----- Point cloud initialisation parameters
        self.INIT_X_NOISE = 10
        self.INIT_Y_NOISE = 10
        self.INIT_Z_NOISE = 0 # We are only in 2D
        self.INIT_Q_NOISE = 0.2

        # ----- Point cloud update parameters
        self.UPDATE_X_NOISE = 0.05
        self.UPDATE_Y_NOISE = 0.05
        self.UPDATE_Z_NOISE = 0 
        self.UPDATE_Q_NOISE = 0.05
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        initial_x = initialpose.pose.pose.position.x
        initial_y = initialpose.pose.pose.position.y
        initial_z = initialpose.pose.pose.position.z
        initial_q = initialpose.pose.pose.orientation

        self.particlecloud.poses = [] # Empty the current particle cloud
        # NB this can be reconfigured to have different behaviours (e.g. remove half of the particles, etc.)

        for i in range(self.NUMBER_PREDICTED_READINGS):
            pose_with_noise = Pose()
            pose_with_noise.position.x = initial_x + random.gauss(0, self.INIT_X_NOISE)
            pose_with_noise.position.y = initial_y + random.gauss(0, self.INIT_Y_NOISE)
            pose_with_noise.position.z = initial_z + random.gauss(0, self.INIT_Z_NOISE)
            pose_with_noise.orientation = rotateQuaternion(initial_q, random.gauss(0, self.INIT_Q_NOISE))
            
            self.particlecloud.poses.append(pose_with_noise)
        
        return self.particlecloud
    
    def _systematic_resampling(self, particles, weights, num_particles):
        new_particles = []

        # Generate cdf
        cdf = [weights[0]]

        for i in range(1, num_particles):
            cdf.append(cdf[-1] + weights[i])

        # Initialise threshold
        thresholds = [random.uniform(0, 1/num_particles)]

        # Draw samples
        i = 1
        for j in range(num_particles):
            while(thresholds[j] > cdf[i]):
                i += 1
            new_particles.append(particles[i])
            thresholds.append(thresholds[-1] + 1/num_particles)
    
        return new_particles
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
         
        weights = []

        for p in self.particlecloud.poses:
            weights.append(self.sensor_model.get_weight(scan, p)) 
        
        # Normalise weights to make it a probability distribution
        weights = [weight / sum(weights) for weight in weights]
        sampled_particles = self._systematic_resampling(self.particlecloud.poses, weights, self.NUMBER_PREDICTED_READINGS)
        
        # Add a tiny bit of noise to prevent the point cloud from collapsing
        new_particles = []
        for i in range(self.NUMBER_PREDICTED_READINGS):
            original_pose = sampled_particles[i]
            pose_after_noise = Pose()
            pose_after_noise.position.x = original_pose.position.x + random.gauss(0, self.UPDATE_X_NOISE)
            pose_after_noise.position.y = original_pose.position.y + random.gauss(0, self.UPDATE_Y_NOISE)
            pose_after_noise.position.z = original_pose.position.z + random.gauss(0, self.UPDATE_Z_NOISE)
            pose_after_noise.orientation = rotateQuaternion(original_pose.orientation, random.gauss(0, self.UPDATE_Q_NOISE))
            new_particles.append(pose_after_noise)

        self.particlecloud.poses = new_particles

        # Store the new weight of each new pose
        self.weights = []
        for pose in self.particlecloud.poses:
            self.weights.append(self.sensor_model.get_weight(scan, pose))
        

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        
        '''
        # Method 1: Naive Average
        particles = self.particlecloud.poses
        average_x = sum(particle.position.x for particle in particles) / self.NUMBER_PREDICTED_READINGS
        average_y = sum(particle.position.y for particle in particles) / self.NUMBER_PREDICTED_READINGS
        average_z = sum(particle.position.z for particle in particles) / self.NUMBER_PREDICTED_READINGS
        average_q_w = sum(particle.orientation.w for particle in particles) / self.NUMBER_PREDICTED_READINGS
        average_q_x = sum(particle.orientation.x for particle in particles) / self.NUMBER_PREDICTED_READINGS
        average_q_y = sum(particle.orientation.y for particle in particles) / self.NUMBER_PREDICTED_READINGS
        average_q_z = sum(particle.orientation.z for particle in particles) / self.NUMBER_PREDICTED_READINGS
        average_q = Quaternion(x=average_q_x, y=average_q_y, z=average_q_z, w=average_q_w)

        estimated_pose = Pose()

        estimated_pose.position.x = average_x
        estimated_pose.position.y = average_y
        estimated_pose.position.z = average_z
        estimated_pose.orientation = average_q
        '''
        
        # Method 2: Take the pose with the highest posterior probability
        highest_weight = 0
        best_pose_index = 0

        for i in range(self.NUMBER_PREDICTED_READINGS):
            if self.weights[i] > highest_weight:
                highest_weight = self.weights[i]
                best_pose_index = i
        
        estimated_pose = self.particlecloud.poses[best_pose_index]

        return estimated_pose
