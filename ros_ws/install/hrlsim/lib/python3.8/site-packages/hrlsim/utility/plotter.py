#! /usr/bin/python3

from os import popen
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import style

from importlib import import_module
import numpy as np

import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

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

class Plotter(Node):
    def __init__(self, nrows, ncols, title=None, subtitles=None, xlabels=None, ylabels=None, style_name="seaborn-whitegrid", windowSize=60, maxPoints=300, pos=None, topics=None):
        nodeName = "plotter_"+title
        super().__init__(nodeName)

        self.get_logger().info("Node created")

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

        if pos != None and topics != None:
            self.get_logger().info("Plotting topics: " + str(topics))
            self.plotTopics(pos,topics)

        self.get_logger().info("Entering run")
        self.run()

    def resetTimeCB(self, req: SetBool.Request) -> SetBool.Response:
        """
        Callback for plotter/reset_time service. 
        """
        self.init_time = self.get_clock().now()
        return SetBool.Response(True, "True")

    def clearDataCB(self, req: SetBool.Request) -> SetBool.Response:
        """
        Callback for plotter/clear_data service.
        """
        for plot in self.plots.flatten():
            for cont in plot.topicContainers:
                cont.x = np.empty(0)
                cont.y = np.array([[]], dtype="object")

        return SetBool.Response(True, "True")

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
        rclpy.loginfo("Message type detected as " + msg_type)

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

        cont.sub = rclpy.Subscriber(topic, msg_class, self.deserialized_cb, args)
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
        topic_list = rclpy.get_published_topics()
        topic_list = np.array(topic_list)

        publishedList = np.empty((0,2))
        for topic in topics:
            topicName = topic["name"]
            index = np.where(topic_list == topicName)

            if(index[0].size == 0):
                rclpy.logwarn("Plotter waiting for topic <" + str(topicName) + "> to be published")
                #rclpy.wait_for_message(topicName, rclpy.AnyMsg)

            publishedList = np.append(publishedList, topic_list[index[0],:], 0)

            self._binary_sub[str(self._binary_sub_id)]=rclpy.Subscriber(topic["name"], rclpy.AnyMsg, self.binary_cb, (pos, topic, str(self._binary_sub_id)))
            self._binary_sub_id += 1

        rclpy.sleep(0.5)



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
                    
                    my = np.min(cont.y)
                    My = np.max(cont.y)
                    miny = min(miny, my-1)
                    maxy = max(maxy, My+1)
                    minx = min(minx, xs[0])
                    maxx = max(maxx, max(xs[-1], window))

                    for i in range(0, len(cont.fields)):
                        y = cont.y[:,i]
                        idy = np.round(np.linspace(0, len(y)-1, self.maxPoints)).astype('int')
                        ys = y[idy]

                        cont.lines[i].set_data(xs, ys)

            plot.ax.set_xlim(minx, maxx)
            plot.ax.set_ylim(miny, maxy)

        time = self.get_clock().now()
        #print(time-self.prevTime)
        self.prevTime = time
        plt.draw()

    def run(self):
        self.get_logger().info("Plotter running")
        self.create_service(SetBool, "plotter/reset_time", self.resetTimeCB)
        self.create_service(SetBool, "plotter/clear_data", self.clearDataCB)
        
        self.ani = animation.FuncAnimation(self.figure, self.drawPlot, interval=50)
        
        self.init_time = self.get_clock().now()
        self.prevTime = self.get_clock().now()
        plt.show()


if __name__ == "__main__":

    from concurrent.futures import ProcessPoolExecutor

    rclpy.init()

    topics = [dict(name="/Team0/Drone0/multirotor",fields=["state.pose.position.x","state.pose.position.y"], styles=["-k","-b"], labels=["x","y"])]

    print("Submitting processes")
    with ProcessPoolExecutor(1) as pool:
        future = pool.submit(Plotter, 2,2,"Test", pos=1, topics=topics)

    