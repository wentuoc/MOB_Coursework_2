#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import matplotlib.pyplot as plt

class GrapherNode(Node):
    def __init__(self):
        super().__init__("grapher")
        self.x_gts = []
        self.y_gts = []
        self.x_es = []
        self.y_es = []
        self.x_amcls = []
        self.y_amcls = []
        self.errors = []

        self._ground_truth_subscriber = self.create_subscription(PoseStamped, 
                                                                 "/ground_truth_pose", 
                                                                 self._ground_truth_callback, 
                                                                 1)
        
        
        self._amcl_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, 
                                                              "/amcl_pose",
                                                              self._amcl_pose_callback,
                                                              1)
        
        self._estimated_pose_subscriber = self.create_subscription(PoseStamped, 
                                                                 "/estimatedpose", 
                                                                 self._estimated_pose_callback, 
                                                                 1)
    
        plt.ion()
        self.fig, (self.ax_xy, self.ax_err) = plt.subplots(1, 2, figsize=(12,5))

        self.timer = self.create_timer(0.1, self.update_graph)
        
    def _ground_truth_callback(self, ground_truth: PoseStamped):
        self.x_gts.append(ground_truth.pose.position.x)
        self.y_gts.append(ground_truth.pose.position.y)

    def _amcl_pose_callback(self, amcl_pose: PoseWithCovarianceStamped):
        self.x_amcls.append(amcl_pose.pose.pose.position.x)
        self.y_amcls.append(amcl_pose.pose.pose.position.y)
    
    def _estimated_pose_callback(self, estimated_pose: PoseStamped):
        self.x_es.append(estimated_pose.pose.position.x)
        self.y_es.append(estimated_pose.pose.position.y)

        if self.x_gts and self.y_gts: # To improve: the estimated pose and ground truth pose may not arrive synchronously
            # Hence, the latest element in each list may not correspond to data at the same point in time
            # Compare the time in the headers to get a better time estimate,
            # and ensure that the times are consistent across the other nodes
            error = ((self.x_es[-1] - self.x_gts[-1]) ** 2 + (self.y_es[-1] - self.y_gts[-1]) ** 2) ** 0.5
            self.errors.append(error)


    def update_graph(self):
        self.ax_xy.clear()
        self.ax_err.clear()
        
        if self.x_es:
            self.ax_xy.set_title("Trajectories")
            self.ax_xy.set_xlabel("X")
            self.ax_xy.set_ylabel("Y")

            self.ax_xy.plot(self.x_gts, self.y_gts, linewidth=1.0, color='blue',  linestyle='-.')
            self.ax_xy.plot(self.x_es, self.y_es, linewidth=1.0, color='magenta')
            self.ax_xy.plot(self.x_amcls, self.y_amcls, linewidth=1.0, color='green', linestyle='--')

            self.ax_err.set_title("Error Over Time")
            self.ax_err.set_xlabel("Time")
            self.ax_err.set_ylabel("Error (m)")

            self.ax_err.plot(range(len(self.errors)), self.errors)

        plt.pause(0.001)
    

def main(args=None):
    rclpy.init(args=args)
    node = GrapherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()