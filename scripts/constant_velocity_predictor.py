#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from ros_gym_sfm.msg import Actor, PredictedActor

class ConstantVelocityPredictor:
    def __init__(self):
        # parameter
        self.HZ = rospy.get_param("/HZ")
        self.radius = rospy.get_param("/actor/radius")
        self.PREDICT_TIME = rospy.get_param("/PREDICT_TIME")
        self.DT = rospy.get_param("/DT")

        # create instance
        self.actors = Actor()  # actor info

        # create callback flag
        self.actor_callback_flag = False  # actor info
        
        # publisher
        self.moving_obs_pub =rospy.Publisher("ros_gym_sfm/prediction", PredictedActor, queue_size=1)
        self.moving_obs_debug_pub =rospy.Publisher("ros_gym_sfm/prediction_debug", MarkerArray, queue_size=1)

        # subscriber
        rospy.Subscriber("ros_gym_sfm/actor_info", Actor, self.actor_callback)

    # callback
    def actor_callback(self, actors):
        self.callback_counter = self.callback_counter + 1
        self.actors= actors
        self.actor_callback_flag = True

        # callback timer for velocity calculation
            # check that the while loop has rotated
        self.previous_loop_count = self.current_loop_count
        self.current_loop_count = self.while_loop_counter 
        if self.current_loop_count > self.previous_loop_count:
            self.previous_actor_time_stamp = self.current_actor_time_stamp
        self.current_actor_time_stamp = actors.header.stamp

    # calculate actors velocity [m/s]
    def calc_vel(self, actors: Actor) -> Actor:
        # store previous pose and current pose
        if self.first_catch_flag == False:
            self.previous_actors = actors
            self.first_catch_flag = True
        else:
            self.previous_actors = self.current_actors
        self.current_actors = actors

        # make velocity
        vels = Actor()
        dt = 0
        if self.current_actor_time_stamp != 0 and self.previous_actor_time_stamp != 0:
            dt = (self.current_actor_time_stamp - self.previous_actor_time_stamp).to_sec()

        if dt != 0:
            for i in range(len(self.current_actors.name.data)):
                vel_i = Point()      

                for j in range(len(self.previous_actors.name.data)):  
                    if self.current_actors.name.data[i] == self.previous_actors.name.data[j]:
                        dx = (self.current_actors.pose.points[i].x - self.previous_actors.pose.points[j].x)
                        dy = (self.current_actors.pose.points[i].y - self.previous_actors.pose.points[j].y)
                        dz = (self.current_actors.pose.points[i].z - self.previous_actors.pose.points[j].z)

                        vel_i.x = dx / dt 
                        vel_i.y = dy / dt  
                        vel_i.z = dz / dt  

                        vels.pose.points.append(vel_i)   
                    
                    elif j == len(self.previous_actors.name.data) - 1 and vel_i.x == 0 and vel_i.y == 0:
                        vels.pose.points.append(vel_i)
        #                 print("actor%d" %self.current_actors.name.data[i], ": First observation!") 
            
        #         if len(self.previous_actors.name.data) == 0:
        #             print("actor%d" %self.current_actors.name.data[i], ": First observation!") 
   
        # if len(self.current_actors.name.data) == 0:
        #     print("No observation ...")

        # print("prev:", self.previous_actors.name.data)
        # print("curr:", self.current_actors.name.data)
        # print(vels.pose.points)

        return vels

    # predict actor future position and create obs_list of time
    def predict_obs(self, actor_pose: Marker, actor_vel: Marker, i: int) -> Marker:
        obs_list_t = Marker()

        if len(actor_pose.points) == len(actor_vel.points):
            for j in range(len(actor_pose.points)):
                obs_list_j = Point()

                obs_list_j.x = actor_pose.points[j].x + actor_vel.points[j].x * self.DT*(i+1)
                obs_list_j.y = actor_pose.points[j].y + actor_vel.points[j].y * self.DT*(i+1)
                obs_list_j.z = actor_pose.points[j].z + actor_vel.points[j].z * self.DT*(i+1)

                obs_list_t.points.append(obs_list_j)
        
        return obs_list_t

    def process(self):
        self.rate = rospy.Rate(self.HZ)
        self.callback_counter = 0        # count the number of times callback is called
        previous_callback_count = 0
        current_callback_count = 0
        self.while_loop_counter = 0     
        self.previous_loop_count = 0    
        self.current_loop_count = 0  
        self.previous_actor_time_stamp = 0  # [sec]
        self.current_actor_time_stamp = 0   # [sec]  
        actors_vel = Actor()              # actor velocity [m/s]
        self.first_catch_flag = False    # flag to receive actor pose for the first time
        self.previous_actors = Actor()
        self.current_actors = Actor() 

        while not rospy.is_shutdown():
            if self.actor_callback_flag == True:
                self.while_loop_counter = self.while_loop_counter + 1
                previous_callback_count = current_callback_count
                current_callback_count = self.callback_counter

                if current_callback_count > previous_callback_count:
                    actors_vel = self.calc_vel(self.actors)
                #     print("actors_name:", self.actors.name.data)
                #     print("vel:", actors_vel.pose.points)
                #     print("Calculate actors velocity!")
                # else:
                #     print("Does not calculate actors velocity")

                obs_lists = PredictedActor()
                obs_lists.name = self.actors.name
                split = int(self.PREDICT_TIME / self.DT)
                for i in range(split):
                    obs_list = Marker()
                    obs_list.header.frame_id ="map"
                    obs_list.header.stamp = rospy.Time.now()
                    obs_list.color.r = 0
                    obs_list.color.g = 0.5
                    obs_list.color.b = 1
                    obs_list.color.a = 1 - i / split  # become light 
                    # obs_list.color.a = (i + 1) / split  # become deep 
                    obs_list.ns = "ros_gym_sfm/prediction"
                    obs_list.id = i
                    obs_list.type = Marker.SPHERE_LIST
                    obs_list.action = Marker.ADD
                    obs_list.lifetime = rospy.Duration()
                    obs_list.scale.x = self.radius
                    obs_list.scale.y = self.radius
                    obs_list.scale.z = self.radius
                    pose = Pose()
                    pose.orientation.w = 1
                    obs_list.pose = pose
                    obs_list_t = Marker()
                    obs_list_t = self.predict_obs(self.actors.pose, actors_vel.pose, i)
                    obs_list.points = obs_list_t.points
                    obs_lists.timepose.markers.append(obs_list)
                self.moving_obs_pub.publish(obs_lists)
                self.moving_obs_debug_pub.publish(obs_lists.timepose)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("constant_velocity_predictor", anonymous=True)

    constant_velocity_predictor = ConstantVelocityPredictor()
    try:
       constant_velocity_predictor .process()
    except rospy.ROSInterruptException:
        pass

