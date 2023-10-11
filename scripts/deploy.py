import rospy
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Twist

import numpy as np
import torch as th
from torch.distributions import Independent, Normal
from train.network import Actor
import train.utils as utils

class deployment:
    def __init__(self, config):
        rospy.init_node('policy_deployment')
        self.config = config

        self.obs_dim = config['map_side_block_num']
        self.act_dim = config['act_dim']
        self.map_side_block_num = config['map_side_block_num']
        self.map_len = config['map_len']
        self.actor = Actor(self.obs_dim, self.act_dim).to(config['device'])
        self.actor.load_state_dict(th.load(config['actor_path'], map_location=config['device']))
        self.actor.eval()

        self.laser_sub = rospy.Subscriber('/laser_pc_topic', PointCloud2, self.callback_pointcloud, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.wheel_base = config['wheel_base']
        self.linear_velocity_ratio = config['linear_velocity_ratio']

        self.rate = rospy.Rate(1000 / config['timestep'])
        self.move_cmd = Twist()

    def dist_fn(self, mu, sigma):
        return Independent(Normal(mu, sigma), 1)

    def get_action(self, obs):
        obs = th.FloatTensor(obs).to(self.config['device'])
        with th.no_grad():
            mu, sigma = self.actor(obs)
            dist = self.dist_fn(mu, sigma)
            action = dist.sample()
            action = th.clamp(action, -1, 1)
        return action.cpu().numpy()
    
    def apply_action(self, action):
        speed = action[0]
        angle = action[1]

        linear_velocity = speed 
        turning_radius = self.wheel_base / np.tan(angle)
        angular_velocity = linear_velocity / turning_radius

        self.move_cmd.linear.x = linear_velocity
        self.move_cmd.angular.z = angular_velocity

    def callback_pointcloud(self, msg):
        assert isinstance(msg, PointCloud2)
        points = list(point_cloud2.read_points_list(msg, field_names=("x", "y", "z")))

        grid = np.full((self.map_side_block_num, self.map_side_block_num), -1)
        obs = utils.map_lidar_pointcloud(grid, points, self.map_side_block_num, self.map_len)
        action = self.get_action(obs)
        self.apply_action(action)

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.move_cmd)
            rate.sleep()


if __name__ == '__main__':
    config = {
        'device': 'cuda' if th.cuda.is_available() else 'cpu',
        'actor_path': 'output_radar/ppo_agent/20231006_223910/actor_499.pth',
        'map_side_block_num': 224,
        'map_len': 30,
        'act_dim': 2,
        'wheel_base': 0.324,
        'linear_velocity_ratio': 0.6944,
        'timestep': 96,
    }

    try:
        deploy = deployment(config)
        deploy.spin()
    except rospy.ROSInterruptException:
        pass