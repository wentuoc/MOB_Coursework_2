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
        
        # System states
        self.weights = []
        self.estimated_pose = Pose()
        self.estimated_pose_weight = 0
        self.previous_estimated_pose_weight = 0
        
        # Logger
        self.logger = logger
        self.is_warning_raised = False

        # Other magic numbers
        self.WEIGHT_THRESHOLD = 10

        # ----- Set motion model parameters
        # TO FINE TUNE
        self.ODOM_ROTATION_NOISE = 0.2 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.2 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.2 # Odometry model y axis (side-to-side) noise

        # ----- Sensor model parameters
        self.STANDARD_NUMBER_PREDICTED_READINGS = 20
        self.number_predicted_readings = 20

        # ----- Point cloud initialisation parameters
        self.INIT_X_NOISE = 0.1
        self.INIT_Y_NOISE = 0.1
        self.INIT_Z_NOISE = 0 # We are only in 2D
        self.INIT_Q_NOISE = 0.05

        # ----- Point cloud update parameters
        self.UPDATE_X_NOISE = 0.02 # TODO: fine-tune
        self.UPDATE_Y_NOISE = 0.02
        self.UPDATE_Z_NOISE = 0 
        self.UPDATE_Q_NOISE = 0.01

        # ----- Point cloud regeneration parameters (for lost robots)
        self.regenerate_cloud_noise = 1
        self.regenerate_cloud_angular_noise = 2 # Ensure that all angles are covered
        self.REGENERATE_CLOUD_NOISE_INCREMENT = 0.05
        self.REGENERATE_CLOUD_NOISE_LIMIT = 5
        self.REGENERATE_CLOUD_KEEP_PERCENTAGE = 0.1
        
       
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

        self.particlecloud.poses = []
        self.estimated_pose = initialpose.pose.pose # initialpose is of type PoseWithCovarianceStamped

        for i in range(self.number_predicted_readings):
            pose_with_noise = self._add_noise_to_pose(initialpose.pose.pose,
                                                      self.INIT_X_NOISE, 
                                                      self.INIT_Y_NOISE, 
                                                      self.INIT_Z_NOISE, 
                                                      self.INIT_Q_NOISE)
            self.particlecloud.poses.append(pose_with_noise)
        
        return self.particlecloud
    
    def _add_noise_to_pose(self, original_pose, x_noise, y_noise, z_noise, q_noise):
        """Adds gaussian noise to a specified pose"""
        pose_with_noise = Pose()
        x = original_pose.position.x
        y = original_pose.position.y
        z = original_pose.position.z
        q = original_pose.orientation

        pose_with_noise.position.x = x + random.gauss(0, x_noise)
        pose_with_noise.position.y = y + random.gauss(0, y_noise)
        pose_with_noise.position.z = z + random.gauss(0, z_noise)
        pose_with_noise.orientation = rotateQuaternion(q, random.gauss(0, q_noise))
        return pose_with_noise
        
    def _systematic_resampling(self, particles, weights, num_particles):
        """Performs importance sampling according to the algorithm detailed in the lecture slides"""
        # Normalise weights to make it a probability distribution
        normalised_weights = [weight / sum(weights) for weight in weights]
        new_particles = []

        # Generate cdf
        cdf = [normalised_weights[0]]

        for i in range(1, len(normalised_weights)):
            cdf.append(cdf[-1] + normalised_weights[i])

        # Initialise threshold
        thresholds = [random.uniform(0, 1/num_particles)]

        # Draw samples
        i = 0
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
        
        self._update_weights(scan)

        if max(self.weights) < self.WEIGHT_THRESHOLD:
            # From empirical observations, if the estimated pose is correct, it has a weight of around 14 (with 20 particles).
            # Conversely, if the maximum weight of all the particles is less than 10 (with 20 particles), we can guess that the
            # estimated pose is probably wrong. Hence, we should generate a new particle cloud

            if not self.is_warning_raised:
                self.logger.warn("The robot has lost its position. Re-localising")
                self.is_warning_raised = True

            if self.regenerate_cloud_noise > self.REGENERATE_CLOUD_NOISE_LIMIT: # Hard-code a limit to give up
                self.logger.warn("The robot is unable to localise itself with the given number of particles. Retrying...")
                self.regenerate_cloud_noise = 1
            elif self.estimated_pose_weight <= self.previous_estimated_pose_weight: # We want the weight to keep increasing
                # If not, widen the search area
                self.regenerate_cloud_noise += self.REGENERATE_CLOUD_NOISE_INCREMENT # The noise used for regeneration will keep increasing to widen the search area
            else:
                self.regenerate_cloud_noise = 1
            self._regenerate_new_particle_cloud(scan)
        else:
            self.regenerate_cloud_noise = 1
            self._update_particle_cloud_with_sensor_model()

            if self.is_warning_raised:
                self.is_warning_raised = False
                self.logger.info("Location re-aquired")
        
        self._update_weights(scan)

    def _update_weights(self, scan):
        """Get and store the weights of the poses in the particle cloud"""
        # Store the new weight of each new pose
        self.weights = self._get_weights(self.particlecloud.poses, scan)
        
    def _get_weights(self, poses, scan):
        """Get the weights given a set of poses and scan data"""
        weights = []
        for pose in poses:
            weights.append(self.sensor_model.get_weight(scan, pose))
        return weights

    def _update_particle_cloud_with_sensor_model(self):
        """Update the particle cloud by resampling based on the poses' weights"""
        # Adaptive resampling based on the value of the current weights
        # Naive method: if the average of current weights are high, we need less samples
        # If the average of the current weights are low, we need more samples
        self.number_predicted_readings = math.ceil(2000 / (sum(self.weights) / len(self.weights)) ** 2)

        sampled_particles = self._systematic_resampling(self.particlecloud.poses, self.weights, self.number_predicted_readings)
        
        # Add a tiny bit of noise to prevent the point cloud from collapsing
        new_particles = []
        for i in range(self.number_predicted_readings):
            original_pose = sampled_particles[i]
            pose_after_noise = self._add_noise_to_pose(original_pose,
                                                    self.UPDATE_X_NOISE, 
                                                    self.UPDATE_Y_NOISE, 
                                                    self.UPDATE_Z_NOISE, 
                                                    self.UPDATE_Q_NOISE)
            new_particles.append(pose_after_noise)
        
        self.particlecloud.poses = new_particles

    def _regenerate_new_particle_cloud(self, scan):
        # Naive method: randomly remove 90% of the particles and add them back, scattered over the map
        num_particles_to_keep = int(self.number_predicted_readings * self.REGENERATE_CLOUD_KEEP_PERCENTAGE)
        new_particles = random.sample(self.particlecloud.poses, num_particles_to_keep)
        new_particles.append(self.estimated_pose) # Ensure that the previous estimated_pose (without any noise) 
        # is always in the set. This ensures that our weight increases

        
        # Adaptive resampling based on the value of the current weights
        # Naive method: if the average of current weights are high, we need less samples
        # If the average of the current weights are low, we need more samples
        self.number_predicted_readings = math.ceil(2000 / (sum(self.weights) / len(self.weights)) ** 2)
        num_particles_to_regenerate = self.number_predicted_readings - num_particles_to_keep
        
        for i in range(num_particles_to_regenerate - 1):
            # As we don't know where the robot is, our best bet is to add points around the current estimated pose
            # (which is the pose with the highest posterior probabiity)
            new_particle = self._add_noise_to_pose(self.estimated_pose,
                                                   self.regenerate_cloud_noise,
                                                   self.regenerate_cloud_noise,
                                                   0, 
                                                   self.regenerate_cloud_angular_noise) 
            new_particles.append(new_particle)

        new_weights = self._get_weights(new_particles, scan)
        new_particles_sampled = self._systematic_resampling(new_particles, new_weights, self.number_predicted_readings)
        
        self.particlecloud.poses = new_particles_sampled

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
        average_x = sum(particle.position.x for particle in particles) / self.number_predicted_readings
        average_y = sum(particle.position.y for particle in particles) / self.number_predicted_readings
        average_z = sum(particle.position.z for particle in particles) / self.number_predicted_readings
        average_q_w = sum(particle.orientation.w for particle in particles) / self.number_predicted_readings
        average_q_x = sum(particle.orientation.x for particle in particles) / self.number_predicted_readings
        average_q_y = sum(particle.orientation.y for particle in particles) / self.number_predicted_readings
        average_q_z = sum(particle.orientation.z for particle in particles) / self.number_predicted_readings
        average_q = Quaternion(x=average_q_x, y=average_q_y, z=average_q_z, w=average_q_w)

        estimated_pose = Pose()

        estimated_pose.position.x = average_x
        estimated_pose.position.y = average_y
        estimated_pose.position.z = average_z
        estimated_pose.orientation = average_q
        '''
        
        '''
        # Method 2: Take the pose with the highest posterior probability
        highest_weight = 0
        best_pose_index = 0

        for i in range(self.number_predicted_readings):
            if self.weights[i] > highest_weight:
                highest_weight = self.weights[i]
                best_pose_index = i
        
        self.previous_estimated_pose_weight = self.estimated_pose_weight # Push the previous weight into history
        self.estimated_pose = self.particlecloud.poses[best_pose_index]
        self.estimated_pose_weight = highest_weight

        return self.estimated_pose
        '''
        
        # Method 3 Compute the weighted average of the poses within a cluster of the highest-weighted pose
        highest_weight = 0
        best_pose_index = 0

        for i in range(self.number_predicted_readings):
            if self.weights[i] > highest_weight:
                highest_weight = self.weights[i]
                best_pose_index = i

        best_pose = self.particlecloud.poses[best_pose_index]

        count = 0
        sum_of_weights = 0
        estimated_pose_x = 0
        estimated_pose_y = 0
        estimated_pose_z = 0
        estimated_pose_q_angle = 0

        # Find all poses within a 0.05m distance
        for i in range(len(self.particlecloud.poses)):
            pose = self.particlecloud.poses[i]
            weight = self.weights[i]
            distance = math.sqrt((pose.position.x - best_pose.position.x) ** 2
                                 + (pose.position.y - best_pose.position.y) ** 2
                                 + (pose.position.z - best_pose.position.z) ** 2)
            if distance <= 0.05:
                estimated_pose_x += weight * pose.position.x
                estimated_pose_y += weight * pose.position.y
                estimated_pose_z += weight * pose.position.z
                estimated_pose_q_angle += weight * getHeading(pose.orientation)
                sum_of_weights += weight
        
        estimated_pose_x /= sum_of_weights
        estimated_pose_y /= sum_of_weights
        estimated_pose_z /= sum_of_weights
        estimated_pose_q_angle /= sum_of_weights
        estimated_pose_q = rotateQuaternion(Quaternion(x=0, y=0, z=0, w=1), estimated_pose_q_angle)
        
        estimated_pose = Pose()
        estimated_pose.position.x = estimated_pose_x
        estimated_pose.position.y = estimated_pose_y
        estimated_pose.position.z = estimated_pose_z
        estimated_pose.orientation = estimated_pose_q

        self.previous_estimated_pose_weight = self.estimated_pose_weight # Push the previous weight into history
        self.estimated_pose = estimated_pose
        return self.estimated_pose

