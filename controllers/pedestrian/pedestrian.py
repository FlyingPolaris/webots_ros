# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pedestrian class container."""
from controller import Supervisor

import optparse
import numpy as np
import math


class Pedestrian(Supervisor):
    """Control a Pedestrian PROTO."""
    
    def __init__(self):
        """Constructor: initialize constants."""
        
        self.root_rotation_field = None
        self.waypoints_distance = None
        self.root_translation_field = None
        self.root_node_ref = None
        self.waypoints = None
        self.time_step = None
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.number_of_waypoints = 100
        self.speed = 1.15
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [
            # those coefficients are empirical coefficients which result in a realistic walking gait
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [  # those coefficients are empirical coefficients which result in a realistic walking gait
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # left hand
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # right lower arm
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # right hand
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # head
        ]
        Supervisor.__init__(self)

    def run(self):
        """Set the Pedestrian pose and position."""
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--trajectory", default="-3 -6, 0 -6, 2 -6",
                              help="Specify the trajectory in the format [x1 y1, x2 y2, ...]")
        opt_parser.add_option("--speed", type=float, default=0.25, help="Specify walking speed in [m/s]")
        opt_parser.add_option("--step", type=int, help="Specify time step (otherwise world time step is used)")
        opt_parser.add_option("--range", type=str, default="0, 0, 0, 7, 4.5, 7",
                              help="the area the pedestrian walking, in the format [x, y, x_min, x_max, y_min, y_max]")
        options, args = opt_parser.parse_args()
        # if not options.trajectory or len(options.trajectory.split(',')) < 2:
        #     print("You should specify the trajectory using the '--trajectory' option.")
        #     print("The trajectory should have at least 2 points.")
        #     return
        if options.speed and options.speed > 0:
            self.speed = options.speed
        if options.step and options.step > 0:
            self.time_step = options.step
        else:
            self.time_step = int(self.getBasicTimeStep())
        # point_list = options.trajectory.split(',')

        # self.number_of_waypoints = len(point_list)
        self.number_of_waypoints = 5

        # 生成行人的轨迹
        range_list = options.range.split(',')
        x_pos = float(range_list[0])
        y_pos = float(range_list[1])
        x_low = float(range_list[2])
        x_high = float(range_list[3])
        y_low = float(range_list[4])
        y_high = float(range_list[5])
        step_size = 2.5

        traj_x = np.random.uniform(x_low, x_high)
        traj_y = np.random.uniform(y_low, y_high)

        self.waypoints = []
        self.waypoints.append([x_pos, y_pos])
        self.waypoints.append([x_low, y_low])
        self.waypoints.append([x_high, y_low])
        self.waypoints.append([x_high, y_high])
        self.waypoints.append([x_low, y_high])



        #
        # for i in range(0, self.number_of_waypoints):
        #     self.waypoints.append([])
        #     self.waypoints[i].append(traj_x)
        #     self.waypoints[i].append(traj_y)
        #     next_x_low = x_low if traj_x - step_size < x_low else traj_x - step_size
        #     next_x_high = x_high if traj_x + step_size > x_high else traj_x + step_size
        #     next_y_low = y_low if traj_y - step_size < y_low else traj_y - step_size
        #     next_y_high = y_high if traj_y + step_size > y_high else traj_y + step_size
        #     traj_x = np.random.uniform(next_x_low, next_x_high)
        #     traj_y = np.random.uniform(next_y_low, next_y_high)
            # print(self.waypoints)
        
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))
        
        # compute waypoints distance
        self.waypoints_distance = []
        for i in range(0, self.number_of_waypoints):
            x = self.waypoints[i][0] - self.waypoints[(i + 1) % self.number_of_waypoints][0]
            y = self.waypoints[i][1] - self.waypoints[(i + 1) % self.number_of_waypoints][1]
            if i == 0:
                self.waypoints_distance.append(math.sqrt(x * x + y * y))
            else:
                self.waypoints_distance.append(self.waypoints_distance[i - 1] + math.sqrt(x * x + y * y))
        
        while not self.step(self.time_step) == -1:
            time = self.getTime()
            
            current_sequence = int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
            # compute the ratio 'distance already covered between way-point(X) and way-point(X+1)'
            # / 'total distance between way-point(X) and way-point(X+1)'
            ratio = (time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - \
                    int(((time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))
            
            for i in range(0, self.BODY_PARTS_NUMBER):
                current_angle = self.angles[i][current_sequence] * (1 - ratio) + \
                                self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                self.joints_position_field[i].setSFFloat(current_angle)
            
            # adjust height
            self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                                         self.height_offsets[
                                             (current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
            
            # move everything
            distance = time * self.speed
            relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * \
                                self.waypoints_distance[self.number_of_waypoints - 1]
            
            for i in range(0, self.number_of_waypoints):
                if self.waypoints_distance[i] > relative_distance:
                    break
            
            distance_ratio = 0
            if i == 0:
                distance_ratio = relative_distance / self.waypoints_distance[0]
            else:
                distance_ratio = (relative_distance - self.waypoints_distance[i - 1]) / \
                                 (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
            x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + \
                (1 - distance_ratio) * self.waypoints[i][0]
            y = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + \
                (1 - distance_ratio) * self.waypoints[i][1]
            root_translation = [x, y, self.ROOT_HEIGHT + self.current_height_offset]
            angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1],
                               self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0])
            rotation = [0, 0, 1, angle]
            
            self.root_translation_field.setSFVec3f(root_translation)
            self.root_rotation_field.setSFRotation(rotation)


controller = Pedestrian()
controller.run()
