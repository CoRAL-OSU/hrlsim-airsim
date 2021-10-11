#! /usr/bin/python3

import hrlsim
import sys

import numpy as np

if __name__ == "__main__":
    if len(sys.argv) != 2:
        drone_name = "Drone0"

    else:
        drone_name = str(sys.argv[1])


    '''
    zoo = hrlsim.opt.ZOO(eta=5e-4*np.ones((2,1)), delta=1e-3)

    def f(x):
        return x.T@np.diag([1,1])@x

    
    x = np.random.randn(2,1)*1


    for i in range(0, 100000):
        print(*x)
        (x, step) = zoo.computeStep(f(x), x)
        if np.linalg.norm(step) > 10:
            break
    '''


    drone = hrlsim.drone.Agent("Team0", drone_name, controllerType=hrlsim.controller.LQR, trajType=hrlsim.traj.MinimumSnap)
    drone.start()

    #plotter = hrlsim.Plotter(nrows=1, ncols=2, windowSize=15, title="Trajectory components", subtitles=["X & Y Components", "Z Component"], xlabels=["t (Seconds)", "t (Seconds)"], ylabels=["m (Meters)", "m (Meters)"])
    
    #plotter.plotTopics(pos=1, topics=[dict(name="/Team0/Drone0/multirotor",fields=["state.pose.position.x","state.pose.position.y"], styles=["-k","-b"], labels=["x","y"]),
    #                                  dict(name="/Team0/Drone0/lqr/desired_pose",fields=["pose.position.x","pose.position.y"], styles=["--k", "--b"], labels=["x0","y0"])])
    #plotter.plotTopics(pos=2, topics=[dict(name="/Team0/Drone0/multirotor",fields=["state.pose.position.z"], styles=["-k"], labels=["z"]),
    #                                  dict(name="/Team0/Drone0/lqr/desired_pose",fields=["pose.position.z"], styles=["--k"], labels=["z0"])])
    
    #plotter.start()
    #drone.join()
    #plotter.join()