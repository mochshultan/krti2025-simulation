from krti2023.drone_api import DroneAPI
from krti2023.msg import QRResult, DResult
from krti2023.srv import Activate, ActivateRequest, ActivateResponse
import rospy
from time import sleep


class Game:
    def __init__(self):
        rospy.init_node("tes_main",log_level=rospy.DEBUG)

        # waypoints = [
        #     {"x": -1, "y": 0, "z": 1.4},  # maju
        #     {"x": -4.6, "y": 0, "z": 1.4},  # 
        #     {"x": -4.6, "y": -5.6, "z": 1.4},  # 
        #     {"x": -3, "y": -5.6, "z": 1.4},  # 
        # ]

        self.drone = DroneAPI()

    
    def test_cmd_vel(self):
        r = rospy.Rate(20)
        rospy.set_param("/mavros/setpoint_velocity/mav_frame", "BODY_NED")
        while not rospy.is_shutdown():
            self.drone.wait4start()
            self.drone.takeoff(2)
            # try:
            now = rospy.Time.now()
            rospy.loginfo("[MISSION] Moving 1 m on x axis")
            self.drone.move_vel({'x': 1, 'y': 0, 'z': 0})
            rospy.loginfo("Finished in %s", rospy.Time.now() - now)
            rospy.sleep(3)
            # sleep(5)
            
            now = rospy.Time.now()
            rospy.loginfo("[MISSION] Moving 1 m on y axis")


            self.drone.move_vel({'x': 0, 'y': -3, 'z': 0})
            # rospy.sleep(3)
            rospy.loginfo("Finished in %s", rospy.Time.now() - now)
            # sleep(5)
            rospy.sleep(3)

            now = rospy.Time.now()
            rospy.loginfo("[MISSION] Moving 1 m on z axis")
            self.drone.move_vel({'x': 0, 'y': 0, 'z': 1}) 
            rospy.sleep(3)
            rospy.loginfo("Finished in %s", rospy.Time.now() - now)

            rospy.loginfo("[MISSION] LAND")
            self.drone.set_mode("LAND")
            
            rospy.signal_shutdown("Finished")

            # rospy.loginfo("[MISSION] Moving 1 m on y axis")
            # self.drone.move_vel({'x': 0, 'y': 1, 'z': 0})
            # rospy.sleep(3)
            # rospy.loginfo("[MISSION] Moving -1 m on y axis")
            # self.drone.move_vel({'x': 0, 'y': -1, 'z': 0})
            # rospy.sleep(3)
            # rospy.loginfo("[MISSION] Moving -1 m on z axis")
            # self.drone.move_vel({'x': 0, 'y': 0, 'z': -1})
            # rospy.sleep(3)
                # self.drone.move_vel({'x': 0, 'y': 0, 'z': 0})
                # rospy.loginfo("[MISSION] Moving 1 m on x axis")
                # self.drone.move_vel({'x': 0, 'y': 0, 'z': 0})
                # rospy.loginfo("[MISSION] Moving 1 m on x axis")
                # rospy.sleep(3)

            # except KeyboardInterrupt:
            #     self.drone.set_mode("LAND")
            #     exit()
            # except IndexError:
            #     pass
        r.sleep()        


if __name__ == "__main__":
    game = Game()
    try:
        game.test_cmd_vel()
    except KeyboardInterrupt:
        game.drone.set_mode("LAND")
        exit()
    except rospy.ROSInterruptException:
        pass
