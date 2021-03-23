#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from PyQt5.QtGui  import *
from PyQt5.QtCore import *
import rosbag
import rospy
import subprocess
from optparse import OptionParser
from datetime import datetime

class SimplePyQtGUIKit:
    def QuitApp(self):
        QtWidgets.QApplication.quit()

    @classmethod
    def GetFilePath(self,caption="Open File",filefilter="",isApp=False):
        u"""
            "Images (*.png *.xpm *.jpg);;Text files (*.txt);;XML files (*.xml)"
        """

        if not isApp:
          app = QtWidgets.QApplication(sys.argv)
        files=QtWidgets.QFileDialog.getOpenFileNames(caption=caption,filter=filefilter)

        strlist=[]
        for file in files:
            strlist.append(str(file))

        return strlist


    @classmethod
    def GetCheckButtonSelect(self, selectList, title="Select", msg="",app=None):
        """
        Get selected check button options

        title: Window name
        mag: Label of the check button
        return selected dictionary
            {'sample b': False, 'sample c': False, 'sample a': False}
        """
 
        if app is None:
          app = QtWidgets.QApplication(sys.argv)
        win = QtWidgets.QWidget()
        scrollArea = QtWidgets.QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollAreaWidgetContents = QtWidgets.QWidget(scrollArea)
        scrollAreaWidgetContents.setGeometry(QRect(0, 0, 380, 247))
        scrollArea.setWidget(scrollAreaWidgetContents)
        layout=QtWidgets.QGridLayout()
        verticalLayoutScroll = QtWidgets.QVBoxLayout(scrollAreaWidgetContents)
        layoutIndex=0

        if msg is not "":
            label = QtWidgets.QLabel(msg)
            layout.addWidget(label,layoutIndex,0)
            layoutIndex=layoutIndex+1

        checkboxs=[]
        for select in selectList:
            checkbox=QtWidgets.QCheckBox(select)
            verticalLayoutScroll.addWidget(checkbox)
            layoutIndex=layoutIndex+1
            checkboxs.append(checkbox)

        layout.addWidget(scrollArea)
        btn=QtWidgets.QPushButton("OK")
        btn.clicked.connect(app.quit)
        layout.addWidget(btn,layoutIndex,0)
        layoutIndex=layoutIndex+1


        win.setLayout(layout)
        win.setWindowTitle(title)
        win.show()
        app.exec_()

        result={}
        for (checkbox, select) in zip(checkboxs, selectList):
            result[select]=checkbox.isChecked()

        return result

def message_to_csv(stream, msg, flatten=False):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv(stream, val, flatten)
    except:
        msg_str = str(msg)
        if msg_str.find(",") is not -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
        stream.write("," + msg_str)

def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join([parent_content_name,s]))
    except:
        stream.write("," + parent_content_name)

def format_csv_filename(form, topic_name):
    global seq
    if form==None:
        return "Convertedbag.csv"
    ret = form.replace('%t', topic_name.replace('/','-'))
    ret=ret[1:]
    return ret
 
def bag_to_csv(options, fname):
    try:
        bag = rosbag.Bag(fname)
        streamdict= dict()
        stime = None
        if options.start_time:
            stime = rospy.Time(options.start_time)
        etime = None
        if options.end_time:
            etime = rospy.Time(options.end_time)
    except Exception as e:
        rospy.logfatal('failed to load bag file: %s', e)
        exit(1)

    try:
        filefolder = fname[fname.rfind('/')+1:-4]
        if not os.path.exists(filefolder):
            os.mkdir(filefolder)
        os.chdir(fname[:-4])
        for topic, msg, time in bag.read_messages(topics=options.topic_names,
                                                  start_time=stime,
                                                  end_time=etime):
            if topic in streamdict:
                stream = streamdict[topic]
            else:
                stream = open(format_csv_filename(options.output_file_format, topic),'w')
                streamdict[topic] = stream
                # header
                if options.header:
                    stream.write("time")
                    message_type_to_csv(stream, msg)
                    stream.write('\n')
            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            message_to_csv(stream, msg, flatten=not options.header)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        rospy.logwarn("fail: %s", e)
    finally:
        bag.close()


def GetTopicList(path):
    # print(path)
    bag = rosbag.Bag(path)
    topics = bag.get_type_and_topic_info()[1].keys()
    # print(topics)
    # print(bag.get_type_and_topic_info()[1].values())
    # print(list(bag.get_type_and_topic_info()[1].values())[1][0])
    types=[]
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
        # print(list(bag.get_type_and_topic_info()[1].values())[i][0])
        types.append(list(bag.get_type_and_topic_info()[1].values())[i][0])

    results=[]    
    for to,ty in zip(topics,types):
        results.append(to)

    #  print "GetTopicList result:"
    #  print results
    return results

def main(options):
    app = QtWidgets.QApplication(sys.argv)

    #GetFilePath
    files=SimplePyQtGUIKit.GetFilePath(isApp=True,caption="Select bag file",filefilter="*bag")
    #  print files
    if len(files)<1:
        print("Error:Please select a bag file")
        sys.exit()
    # files[0] = files[0].split('')[1]
    files[0] = files[0][2:-2]
    # print(files[0])
    topics=GetTopicList(files[0])
    selected=SimplePyQtGUIKit.GetCheckButtonSelect(topics,app=app,msg="Select topics to convert csv files")

    options.topic_names=[]
    for k,v in selected.items():
        if v:
            options.topic_names.append(k)

    if len(options.topic_names)==0:
        print("Error:Please select topics")
        sys.exit()

    options.output_file_format="%t.csv"

    print("Converting....")
    bag_to_csv(options,files[0])

    QtWidgets.QMessageBox.information(QtWidgets.QWidget(), "Message", "Finish Convert!!")

if __name__ == '__main__':
    #print(rosbag_to_csv start!!)
    rospy.init_node('rosbag_to_csv', anonymous=True)
    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-a", "--all", dest="all_topics",
            action="store_true",
            help="exports all topics", default=False)
    parser.add_option("-t", "--topic", dest="topic_names",
            action="append",
                      help="white list topic names", metavar="TOPIC_NAME")
    parser.add_option("-s", "--start-time", dest="start_time",
                      help="start time of bagfile", type="float")
    parser.add_option("-e", "--end-time", dest="end_time",
                      help="end time of bagfile", type="float")
    parser.add_option("-n", "--no-header", dest="header",
                      action="store_false", default=True,
                      help="no header / flatten array value")
    (options, args) = parser.parse_args()


    main(options)