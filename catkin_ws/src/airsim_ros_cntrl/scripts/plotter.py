#! /usr/bin/python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import style

from importlib import import_module
import numpy as np

import rospy, threading
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from operator import attrgetter

class TopicContainer:
    def __init__(self):
        self.topic = ""
        self.x = np.empty(0)
        self.y = np.array([[]], dtype="object")
        self.fields = []
        self.lines = []
        self.sub = None

class PlotContanier:
    def __init__(self):
        self.topics = []
        self.topicContainers = []
        self.ax = ""

class Plotter:
    def __init__(self, nrows, ncols, title=None, subtitles=None, xlabels=None, ylabels=None, style_name="seaborn-whitegrid", windowSize=60, maxPoints=300):
        style.use(style_name)
        self.windowSize = windowSize
        self.maxPoints = maxPoints

        self.nrows = nrows
        self.ncols = ncols

        self.plots = np.empty((nrows,ncols), dtype="object")
        self._binary_sub = dict()
        self._binary_sub_id = 0

        
        self.figure, self.axs = plt.subplots(nrows,ncols)
        self.axs = np.reshape(self.axs, (nrows,ncols))

        rospy.Service("plotter/reset_time", SetBool, self.resetTimeCB)
        rospy.Service("plotter/clear_data", SetBool, self.clearDataCB)

        if title != None:
            self.figure.suptitle(title)

        for r in range(0, nrows):
            for c in range(0, ncols):
                pos = r*ncols+c + 1
                self.plots[r,c] = PlotContanier()
                self.plots[r,c].ax = self.axs[r,c]

                if xlabels != None:
                    assert len(xlabels) == nrows*ncols, "Number of xlabels must be equal to number of subplots"
                    self.plots[r,c].ax.set_xlabel(xlabels[pos-1])

                if ylabels != None:
                    assert len(ylabels) == nrows*ncols, "Number of ylabels must be equal to number of subplots"
                    self.plots[r,c].ax.set_ylabel(ylabels[pos-1])

                if subtitles != None:
                    assert len(subtitles) == nrows*ncols, "Number of axis subtitles must be equal to number of subplots"
                    self.plots[r,c].ax.set_title(subtitles[pos-1])

        self.ani = animation.FuncAnimation(self.figure, self.drawPlot, interval=50)
        
        self.init_time = rospy.get_time()
        self.prevTime = rospy.get_time()

    def resetTimeCB(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for plotter/reset_time service. 
        """
        self.init_time = rospy.get_time()
        return SetBoolResponse(True, "True")

    def clearDataCB(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Callback for plotter/clear_data service.
        """
        for plot in self.plots.flatten():
            for cont in plot.topicContainers:
                cont.x = np.empty(0)
                cont.y = np.array([[]], dtype="object")

        return SetBoolResponse(True, "True")

    def binary_cb(self, data, args):
        pos = args[0]
        topic  = args[1]["name"]
        fields = args[1]["fields"]
        styles = args[1]["styles"]
        labels = args[1]["labels"]
        id = args[2]

        c = (pos-1)%self.ncols
        r = int((pos-1)/self.ncols) 

        connection_header = data._connection_header["type"].split("/")
        ros_pkg = connection_header[0] + ".msg"
        msg_type = connection_header[1]
        rospy.loginfo("Message type detected as " + msg_type)

        msg_class = getattr(import_module(ros_pkg), msg_type)


        self._binary_sub[id].unregister()
        self._binary_sub[id] = None


        self.plots[r,c].topics.append(topic)
        cont = TopicContainer()
        cont.topic = topic
        self.plots[r,c].topicContainers.append(cont)

        for i in range(0, len(fields)):
            cont.fields.append(fields[i])
            cont.lines.append(self.axs[r,c].plot([], [], styles[i], linewidth='1.5', label=labels[i])[0])

        cont.y = np.empty((0,len(fields)))        

        self.plots[r,c].ax.legend()

        for plot in self.plots.flatten():
            for oldCont in plot.topicContainers:
                if oldCont.topic == topic and oldCont.sub != None:
                    cont.sub = oldCont.sub
                    return

        cont.sub = rospy.Subscriber(topic, msg_class, self.deserialized_cb, args)
        return

    def deserialized_cb(self, msg, args):
        topic = args[1]["name"] 

        stamp = attrgetter("header.stamp")(msg)
        time = stamp.secs + float(stamp.nsecs)/1.0e9

        time -= self.init_time

        for plot in self.plots.flatten():
            for cont in plot.topicContainers:
                if cont.topic == topic:
                    cont.x = np.append(cont.x, time)
                    fieldVals = np.empty(0)

                    for field in cont.fields:
                        val = attrgetter(field)(msg)
                        fieldVals = np.append(fieldVals, val)

                    cont.y = np.append(cont.y, [fieldVals], 0)

                    if cont.x[-1] - cont.x[0] > self.windowSize:
                        cont.x = np.delete(cont.x, 0)
                        cont.y = np.delete(cont.y, 0, 0)


    def plotTopics(self, pos, topics):
        topic_list = rospy.get_published_topics()
        topic_list = np.array(topic_list)

        publishedList = np.empty((0,2))
        for topic in topics:
            topicName = topic["name"]
            index = np.where(topic_list == topicName)

            if(index[0].size == 0):
                rospy.logwarn("Plotter waiting for topic <" + str(topicName) + "> to be published")
                rospy.wait_for_message(topicName, rospy.AnyMsg)

            publishedList = np.append(publishedList, topic_list[index[0],:], 0)

            self._binary_sub[str(self._binary_sub_id)]=rospy.Subscriber(topic["name"], rospy.AnyMsg, self.binary_cb, (pos, topic, str(self._binary_sub_id)))
            self._binary_sub_id += 1

        rospy.sleep(0.5)



    def drawPlot(self, i):
        window = self.windowSize

        for plot in self.plots.flatten():
            minx = miny = 1e9
            maxx = maxy = -1e9

            for cont in plot.topicContainers:
                x = cont.x

                if(len(x) > 0):
                    idx = np.round(np.linspace(0, len(x)-1, self.maxPoints)).astype('int')
                    xs = x[idx]
                    
                    miny = min(miny, np.min(cont.y)-1)
                    maxy = max(maxy, np.max(cont.y)+1)
                    minx = min(minx, xs[0])
                    maxx = max(maxx, max(xs[-1], window))

                    for i in range(0, len(cont.fields)):
                        y = cont.y[:,i]
                        idy = np.round(np.linspace(0, len(y)-1, self.maxPoints)).astype('int')
                        ys = y[idy]

                        cont.lines[i].set_data(xs, ys)

            plot.ax.set_xlim(minx, maxx)
            plot.ax.set_ylim(miny, maxy)

        time = rospy.get_time()
        #print(time-self.prevTime)
        self.prevTime = time
        plt.draw()





if __name__ == "__main__":

    rospy.init_node("plotter")

 



    plotter = Plotter(nrows=1, ncols=2, windowSize=45, title="Trajectory components", subtitles=["X & Y Components", "X Component"], xlabels=["t (Seconds)", "t (Seconds)"], ylabels=["m (Meters)", "m (Meters)"])

    
    #th = threading.Thread(target=plt.show(), daemon=True)
    #th.start()

    plotter.plotTopics(pos=1, topics=[dict(name="/Team0/Drone0/multirotor",fields=["state.pose.position.x","state.pose.position.y"], styles=["-k","-b"], labels=["x","y"]),
                                      dict(name="/Team0/Drone0/lqr/desired_pose",fields=["pose.position.x","pose.position.y"], styles=["--k", "--b"], labels=["x0","y0"])])
    plotter.plotTopics(pos=2, topics=[dict(name="/Team0/Drone0/multirotor",fields=["state.pose.position.z"], styles=["-k"], labels=["z"]),
                                      dict(name="/Team0/Drone0/lqr/desired_pose",fields=["pose.position.z"], styles=["--k"], labels=["z0"])])
    plt.show()
