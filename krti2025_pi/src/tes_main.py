from krti2023_pi.drone_api import DroneAPI
import rospy
from time import sleep


class Game:
    def __init__(self):
        rospy.init_node("main_node", log_level=rospy.DEBUG)

        waypoints = [
            #  x = - forward, y = + right, z = + up 

            {"x": -1, "y": 0, "z": 2.4},  # maju
            {"x": -4.6, "y": 0, "z": 2.4},  # 
            {"x": -4.6, "y": -5.6, "z": 2.4},  # 
            {"x": -3, "y": -5.6, "z": 0},  # 
            
        ]

        self.drone = DroneAPI(waypoints=waypoints)

    

    # def land_algorithm(self):
    #     now = rospy.Time.now()
    #     x_done = False
    #     y_done = False

    #     while self.elp_data.is_found:
    #         cur_pose = {
    #             "x": self.drone.current_pose.pose.pose.position.x,
    #             "y": self.drone.current_pose.pose.pose.position.y,
    #             "z": self.drone.current_pose.pose.pose.position.z,
    #         }

    #         if self.elp_data.dx > 20:
    #             print("move right")
    #             cur_pose["x"] += 0.1
    #         elif self.elp_data.dx < -20:
    #             print("move left")
    #             cur_pose["x"] -= 0.1
    #         else:
    #             x_done = True
    #         if self.elp_data.dy > 20:
    #             print("move backward")
    #             cur_pose["y"] -= 0.1

    #         elif self.elp_data.dy < -20:
    #             print("move forward")
    #             cur_pose["y"] += 0.1
    #         else:
    #             y_done = True
    #         if x_done and y_done and cur_pose["z"] > 1.2:
    #             cur_pose["z"] = 1
    #             self.drone.move(cur_pose)
    #             continue
    #         elif x_done and y_done and cur_pose["z"] < 1.2 and cur_pose["z"] > 0.6:
    #             cur_pose["z"] = 0.5
    #         elif x_done and y_done and cur_pose["z"] < 0.3:
    #             rospy.loginfo("landing on elp complete activating land mode")
    #             self.drone.set_mode("LAND")
    #             return 0

    #         if rospy.Time.now() - now > rospy.Duration(0.2):
    #             cur_pose["z"] -= 0.15
    #             now = rospy.Time.now()

    #         print(cur_pose)
    #         self.drone.move(cur_pose)
    #         sleep(0.5)
    #         print("landing elp")
    #     return 1

    def main(self):

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
           
            self.drone.wait4start()
            self.drone.takeoff(1.3)
            try:
                start = rospy.Time.now()
                

                self.drone.set_mode("LAND")

                rospy.signal_shutdown("done")


            except KeyboardInterrupt:
                self.drone.set_mode("LAND")
                exit()
            except IndexError:
                pass
        r.sleep()
        self.drone.set_mode("LAND")
        rospy.signal_shutdown("Mission Finished")




if __name__ == "__main__":
    game = Game()
    try:
        game.main()
    except KeyboardInterrupt:
        game.drone.set_mode("LAND")
        exit()
    except rospy.ROSInterruptException:
        pass
