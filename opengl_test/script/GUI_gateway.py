#!/usr/bin/env python
import random
import string
import json
import cherrypy
#
import rospy
from opengl_test.msg import * # test


# Publishers
GUI_cmd_pub = rospy.Publisher('GUI2/operation', GUI2_op, queue_size=10)
GUI_cmd_pub_seq = 0

GUI_state = GUI2_op()
GUI_state.header.seq = -1
GUI_state_seq = 0
GUI_state_seq_old = 0


class GUI_GATEWAY(object):
    @cherrypy.expose
    def index(self):
        return "Hello world!"

    # --- The following is recommended for POST ---
    # For using POST and read the json through cherrypy.tools.json_in()
    # then reponse a python dict as json though cherrypy.tools.json_out()
    @cherrypy.expose
    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def json_in_out(self):
        j_in = None
        try:
            j_in = cherrypy.request.json
        except:
            print('no json')

        if not j_in is None:
            print("j_in: " + str(j_in))
            # print("j_in['data1'] = %s" % j_in['data1'])
        # Publish to GUI
        ros_msg = GUI2_op()
        ros_msg.header.seq = GUI_cmd_pub_seq
        ros_msg.header.stamp = rospy.get_rostime()
        ros_msg.cam_type = j_in.get("cam_type", "")
        ros_msg.image3D = j_in.get("image3D", "")
        ros_msg.image_surr = j_in.get("image_surr", "")
        ros_msg.cam_motion = j_in.get("cam_motion", "")
        GUI_cmd_pub.publish(ros_msg)
        GUI_cmd_pub_seq += 1

        # Wait for GUI to response
        is_received_GUI_state = False
        start_time = rospy.get_rostime()
        wait_timeout = rospy.Duration.from_sec(1.0) # Wait for 1 sec.
        while (GUI_state_seq == GUI_state_seq_old) and ( (rospy.get_rostime() - start_time) < wait_timeout ):
            rospy.sleep(0.1) # Sleep for 0.1 sec.
        if GUI_state_seq > GUI_state_seq_old:
            is_received_GUI_state = True
        GUI_state_seq_old = GUI_state_seq
        print("is_received_GUI_state = %s" % str(is_received_GUI_state))

        # Output
        res_data = dict()
        res_data["cam_type"] = GUI_state.cam_type
        res_data["image3D"] = GUI_state.image3D
        res_data["image_surr"] = GUI_state.image_surr
        # jdata = json.dumps(data)
        return res_data



def GUI2_state_CB(msg):
    GUI_state = msg
    GUI_state_seq += 1
    print("[GUI-gateway] GUI state recieved.")



if __name__ == '__main__':
    # ROS
    rospy.init_node('GUI_gateway', anonymous=True)
    rospy.Subscriber("GUI2/state", GUI2_op, GUI2_state_CB)

    # HTTP server
    cherrypy.server.socket_host = '0.0.0.0'
    cherrypy.server.socket_port = 6060
    cherrypy.server.thread_pool = 10
    cherrypy.quickstart(GUI_GATEWAY())

    print("End og GUI_gateway")
