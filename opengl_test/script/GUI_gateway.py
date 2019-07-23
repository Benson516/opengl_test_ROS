#!/usr/bin/env python
import random
import string
import json
import cherrypy
import threading
#
import rospy
from std_msgs.msg import Empty
from opengl_test.msg import * # test


# MQTT
#---------------------------#
import paho.mqtt.client as mqtt
#---------------------------#


# ROS Publishers
msg_recorder_trigger_pub = rospy.Publisher('/REC/req_backup', Empty, queue_size=10)
GUI_cmd_pub = rospy.Publisher('GUI2/operation', GUI2_op, queue_size=10)
GUI_cmd_pub_seq = 0

# Global variables
#----------------------------#
# GUI_op responce
GUI_state = GUI2_op()
GUI_state_seq = 0
GUI_state_seq_old = 0
#----------------------------#


# MQTT
#------------------------------------------------------------#
mqtt_client = None
mqtt_broker = "127.0.0.1" # localhost
# MQTT topics
mqtt_msg_topic_namespace = 'REC'

# Subscribed topics (MQTT --> ROS)
#------------------------#
# Alive signal
mqtt_REC_req_backup_subT = mqtt_msg_topic_namespace + '/req_backup'

# The subscription list
# element: (topic, QoS)
mqtt_subscription_list = list()
# NOTE: since that we are subscrbing to local broker, the loading is tiny for QoS=2
mqtt_subscription_list.append( (mqtt_REC_req_backup_subT, 2) )
#------------------------#

# Published topics (MQTT --> dock)
#------------------------#
mqtt_sync_pubT = mqtt_msg_topic_namespace + '/sync' #
#------------------------#

#------------------------------------------------------------#
# end MQTT





# MQTT --> ROS
#------------------------------------------------------#
# The callback for when the client receives a CONNACK response from the server.
def mqtt_on_connect(client, userdata, flags, rc):
    """
    This is the callback function at connecting the MQTT client.
    """
    global mqtt_luggage_dock_namespace

    rospy.loginfo("MQTT client connected with result code <" + str(rc) + ">.")

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe( mqtt_subscription_list )

# The callback for when a PUBLISH message is received from the server.
'''
def mqtt_on_message(client, userdata, mqtt_msg):
    """
    This is the callback function at MQTT messages (un-matched topics) arrival.
    ** On receiving this message, bypass to ROS.
    """
    print(mqtt_msg.topic + " " + str(mqtt_msg.payload))
'''

def mqtt_REC_req_backup_CB(client, userdata, mqtt_msg):
    """
    This is the callback function at receiving req_backup signal.
    ** On receiving this message, bypass to ROS.
    """
    msg_recorder_trigger_pub.publish( Empty() )
#------------------------------------------------------#
# end MQTT --> ROS

# ROS callbacks
#------------------------------------------------------------#
def GUI2_state_CB(msg):
    global GUI_state, GUI_state_seq
    GUI_state = msg
    GUI_state_seq += 1
    print("[GUI-gateway] GUI state recieved.")
#------------------------------------------------------------#
# end ROS callbacks


# HTTP server
#------------------------------------------------------------#
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
        global GUI_cmd_pub, GUI_cmd_pub_seq
        global GUI_state, GUI_state_seq, GUI_state_seq_old
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
        ros_msg.gui_name = j_in.get("gui_name", "")
        ros_msg.cam_view_mode = j_in.get("cam_view_mode", "")
        ros_msg.cam_motion_mode = j_in.get("cam_motion_mode", "")
        ros_msg.image3D = j_in.get("image3D", "")
        ros_msg.image_surr = j_in.get("image_surr", "")
        ros_msg.cam_op = j_in.get("cam_op", "")
        ros_msg.record_op = j_in.get("record_op", "")
        GUI_cmd_pub.publish(ros_msg)
        GUI_cmd_pub_seq += 1
        # Trigger backup
        if ros_msg.record_op == "backup":
            msg_recorder_trigger_pub.publish( Empty() )
        #

        # Wait for GUI to response
        is_received_GUI_state = False
        start_time = rospy.get_rostime()
        wait_timeout = rospy.Duration.from_sec(1.0) # Wait for 1 sec.
        while (GUI_state_seq == GUI_state_seq_old) and ( (rospy.get_rostime() - start_time) < wait_timeout ):
            rospy.sleep(0.01) # Sleep for 0.01 sec.
        if GUI_state_seq > GUI_state_seq_old:
            is_received_GUI_state = True
        GUI_state_seq_old = GUI_state_seq
        print("is_received_GUI_state = %s" % str(is_received_GUI_state))

        # Output
        res_data = dict()
        res_data["gui_name"] = GUI_state.gui_name
        res_data["cam_view_mode"] = GUI_state.cam_view_mode
        res_data["cam_motion_mode"] = GUI_state.cam_motion_mode
        res_data["image3D"] = GUI_state.image3D
        res_data["image_surr"] = GUI_state.image_surr
        # jdata = json.dumps(data)
        return res_data
#------------------------------------------------------------#
# end HTTP server











# Wait for ROS terminating signal to close the HTTP server
def wait_for_close():
    while not rospy.is_shutdown():
        rospy.sleep(0.5)
    cherrypy.engine.exit()

def main():
    #
    global mqtt_client
    # ROS
    rospy.init_node('GUI_gateway', anonymous=True)
    rospy.Subscriber("GUI2/state", GUI2_op, GUI2_state_CB)

    _t = threading.Thread(target=wait_for_close)
    # _t.daemon = True
    _t.start()


    # MQTT
    #-------------------------------------------------------------------#
    mqtt_client = mqtt.Client()
    # Callback functions
    mqtt_client.on_connect = mqtt_on_connect
    # mqtt_client.on_message = mqtt_on_message # We don't need to process the message we don't want

    # Subscriber callbacks
    mqtt_client.message_callback_add(mqtt_REC_req_backup_subT, mqtt_REC_req_backup_CB)
    #-------------------------------------------------------------------#


    # HTTP server
    cherrypy.server.socket_host = '0.0.0.0'
    cherrypy.server.socket_port = 6060
    cherrypy.server.thread_pool = 10
    cherrypy.quickstart(GUI_GATEWAY())

    print("End of GUI_gateway")

if __name__ == '__main__':
    main()
