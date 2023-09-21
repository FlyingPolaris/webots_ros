from controller import *
from ros_utils import *
from multiprocessing import Process, Pool
import globalvar as gl

robot = Supervisor()

timestep = 32*3

# pool = Pool(2)

# rospy.sleep(25)
# pool.apply_async(func=rosservice_lidar_enable, args=(32,))
# p1 = Process(target=rosservice_lidar_enable, args=(32,))
# p1.start()

# rospy.sleep(0.1)
# pool.apply_async(func=rosservice_lidar_pointcloud_enable)

# p2 = Process(target=rosservice_lidar_pointcloud_enable)
# p2.start()

# # rospy.sleep(0.1)
# rosservice_lidar_enable(96)
# rosservice_lidar_pointcloud_enable(True)
# # rospy.sleep(0.1)
# flag = 1

gl._init()
rospy.init_node('touch_sensor_listener', anonymous=True)
touch = threading.Thread(target=rostopic_touch_sensor_get)
touch.start()

pointcloud = threading.Thread(target=rostopic_lidar_pointcloud_get)
pointcloud.start()

while True:
    print('hello ros!')
    # rostopic_touch_sensor_get()
    print(gl.get_value('touch'))
    print(gl.get_value('point_cloud'))

    robot.step(timestep)

    # print(rosservice_lidar_pointcloud_get())
    # pool.close()
    # pool.join()
    # p = Pool()
    # res = p.apply_async(func=rosservice_lidar_pointcloud_get, args=(0,))
    # print(res)
