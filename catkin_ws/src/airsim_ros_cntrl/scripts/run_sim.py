#! /usr/bin/python3

from team import Team
from target import Target

from camera import Camera

import multiprocessing as mp
import airsim, rospy

from std_srvs.srv import SetBool
from geometry_msgs.msg import Vector3
from airsim_ros_cntrl.msg import MoveToLocationGoal

from utilities import createDroneTeamLists



team_list = []


def shutdown() -> None:
    """
    Shuts downs all teams 
    """
    for team in team_list:
        team.shutdown()





if __name__ == "__main__":

    ######################################
    #
    #     SETUP PYTHON CLIENT
    #

    ip = ""  # UNCOMMENT TO RUN ON LOCALHOST
    #ip = "10.0.0.3"  # "192.168.1.129"         # UNCOMMENT TO RUN ON REMOTE HOST

    client = airsim.MultirotorClient(ip=ip)
    client.confirmConnection()

    ######################################
    #
    #     CREATE DRONE/TEAM LISTS

    team_list, drone_list, target_list, target_procs = createDroneTeamLists(client, ip, setupTargets=False)

    ######################################
    #
    #     SETUP ROS

    print("SETUP ROS ON PARENT PROCESS")
    rospy.init_node("swarm")
    rospy.on_shutdown(shutdown)

    windPub = rospy.Publisher("/airsim_node/set_wind_velocity", Vector3, queue_size=10)
    apiSrv = rospy.ServiceProxy("/airsim_node/enable_api", SetBool)


    for i in team_list:
        i.setup_ros()


    ######################################
    #
    #       ENABLE API CONTROL

    try:
        apiSrv(True)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        shutdown()
        exit()
    
    ######################################
    #
    #     SET WIND

    windMsg = Vector3(0,0,0)
    windPub.publish(windMsg) # n,e,d
    print("SET WIND TO [n:{:.1f},e:{:.1f},d:{:.1f}]".format(windMsg.x, windMsg.y, windMsg.z))
    
    ######################################
    #
    #     RUN SIMULATION


    #rospy.sleep(1)
    #camera = Camera("Camera")
    #camera.start()

    
    print("TAKING OFF")
    for team in team_list:
        team.move_to_location([0,0,-3], 1, 20, 0.25, position_frame=MoveToLocationGoal.LOCAL_FRAME)
        #team.takeoff(False)

    #rospy.sleep(5)
    #team_list[0].activateAgents(-5)
    #team_list[1].activateAgents(-10)


    print("MOVE TO LOCATION")
    team_list[0].move_to_location([-10,-8,-5], 2, 30, 0.1)
    #team_list[0].moveInFormation([-245,-255,-5], 2)
    #team_list[1].moveInFormation([-240,-255,-10],2)
    
    
    #team.trackTargetInFormation(20, -10)
    #rospy.sleep(20)


    '''
    print("BEGIN TRACKING")
    team_list[0].track_object(90, -5)
    team_list[1].track_object(90,-10)
    
    #for team in team_list:
    #    team.track_object(40,-4)
    
    for team in team_list:
        team.wait()
    '''



    print("LANDING")
    for team in team_list:
        team.move_to_location([0,0,5],2,20,0.2, position_frame=MoveToLocationGoal.LOCAL_FRAME)
        
    for team in team_list:
        team.land()

    rospy.sleep(10)
    
    
    for t in target_list:
        target_procs[t[0]].shutdown()

    print("SIMULATION SHUTDOWN")
    shutdown()