# Copyright 2024, Christophe VIEL
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted # provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

import subprocess, os, signal
import psutil

import os
import os.path
import pyautogui

import cv2
#from vidgear.gears import WriteGear
from cv_bridge import CvBridge

import gi
#import imutils
import numpy as np


gi.require_version('Gst', '1.0')
from gi.repository import Gst

from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu

from scipy.spatial.transform import Rotation 


class Video:
    """BlueRov video capture class constructor

    #https://gist.github.com/patrickelectric/443645bb0fd6e71b34c504d20d475d5a?permalink_comment_id=3581249

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.resolution = (640, 480)  

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()

        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)

        resolution = (
            caps.get_structure(0).get_value('width'),
            caps.get_structure(0).get_value('height')

        )

        return array, resolution

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf,

            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame, new_resolution = self.gst_to_opencv(sample)
        self._frame = new_frame

        self.resolution = new_resolution

        return Gst.FlowReturn.OK


class Tracker(Node):

    def __init__(self):


        super().__init__('tracking_node')

        self.fps = 15 
        self.queue_size_video = 2 # 10 # que les deux dernières ?
        
        # pour reception des informations

        self.declare_parameter('ROV_name', '')
        self.ROV_name = self.get_parameter('ROV_name').get_parameter_value().string_value
        self.declare_parameter('ID', 1.0)
        ID = self.get_parameter('ID').get_parameter_value().double_value
        self.ID = str(ID)
        self.ID0 = ID 
        self.declare_parameter('nb_ROV', 1.0)
        self.nb_ROV = self.get_parameter('nb_ROV').get_parameter_value().double_value

        self.declare_parameter('camera_port_udp', 5600.0)
        self.port_udp = int(self.get_parameter('camera_port_udp').get_parameter_value().double_value)

        if not(self.ROV_name == ''):
            self.ns = "/"+ self.ROV_name # TODO
        else:
            self.ns = ""

        #self.Affichage = int(rospy.get_param('~affichage_on','1')) # <- la valeur par defaut ne marche pas...
        self.declare_parameter('affichage_on', 1.0)
        self.Affichage = self.get_parameter('affichage_on').get_parameter_value().double_value
        
        self.declare_parameter('num_cam', 1.0)
        self.num_cam = int(self.get_parameter('num_cam').get_parameter_value().double_value)
        self.num_camera = str(self.num_cam)

        self.cam_active = 1.0
        
        self.declare_parameter('display_length_video', 800.0)
        self.display_length_video = self.get_parameter('display_length_video').get_parameter_value().double_value

        
        self.heading = 0.0
        self.altitude = 0.0

        self.Phi, self.Theta, self.Psy = 0.0, 0.0, 0.0
        

        # list publisher
        self.queue_listener = 10
        self.image_pub = self.create_publisher(CompressedImage, self.ns+'/cam_'+ str(self.num_cam) +'/compressed', self.queue_size_video)

        self.output_params = {"-vcodec":"libx264", "-crf": 0, "-preset": "fast"} # define (Codec,CRF,preset) FFmpeg tweak parameters for writer 
        self.resolution = cv2.VideoWriter_fourcc(*'DIVX');


        # Create useful objects
        self.video = Video(self.port_udp)


        while not self.video.frame_available():  # wait for the first frame
            pass

        print("\n---------------- Video OK ------------\n")
        self.frame = self.video.frame()

        self.frame_size = (int(self.video.resolution[0]), int(self.video.resolution[1]))
        self.screen_size = pyautogui.size()
        

        if self.Affichage == 1:
            cv2.namedWindow("Camera "+self.ROV_name+" "+str(self.num_cam))
            cv2.moveWindow("Camera "+self.ROV_name+" "+str(self.num_cam), int(self.ID0-1)*int(self.screen_size[0]/self.nb_ROV)+1 + int((self.display_length_video+1)*int(self.num_cam-1))+50, 1) # note: le +50 pour la largeur des barres outils de linux
        
        self.tracked = False
        self.bbox_detected = None


        # listener
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.queue_listener = 10
        self.listener()
        ###########


        timer_period = 1/self.fps # seconds
        self.timer = self.create_timer(timer_period, self.run)
        


    def listener(self):
 
        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/compass_hdg',
            self.callback_compass,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/mavros/global_position/rel_alt',
            self.callback_alt,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Float64,
            self.ns+'/cam_active',
            self.callback_cam_active,
            self.queue_listener)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Imu,
            self.ns+"/mavros/imu/data",
            self.callback_imu,
            self.qos_profile)
            #self.queue_listener)
        self.subscription  # prevent unused variable warning


        
    def callback_imu(self, msg):
        self.angular_velocity_z = msg.angular_velocity.z  # vitesse de lacet
        self.angular_velocity_y = msg.angular_velocity.y  # (vitesse de tangage ?)
        self.angular_velocity_x = msg.angular_velocity.x  # (vitesse de roulis ?)
        W = msg.orientation.w
        X = msg.orientation.x
        Y = msg.orientation.y
        Z = msg.orientation.z
        orientq=(W, X, Y, Z)
        ### Conversion quaternions in rotation matrix
        Phi_rad, Theta_rad, Psy_rad = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet     
        self.Phi, self.Theta, self.Psy = Phi_rad , Theta_rad, Psy_rad     
            
    def callback_compass(self,data):
        self.heading = data.data

    def callback_alt(self,data):
        self.altitude = -data.data

    def callback_cam_active(self,data):
        self.cam_active = data.data                       	



########################################################


    def publisher_image(self):
    
        #https://python.hotexamples.com/fr/examples/cv_bridge/CvBridge/cv2_to_compressed_imgmsg/python-cvbridge-cv2_to_compressed_imgmsg-method-examples.html?utm_content=cmp-true
        msg = CompressedImage()
        #pub = rospy.Publisher(self.ns+'/cam_'+ str(self.num_cam) +'/compressed', CompressedImage, queue_size=self.queue_size_video)
        
        bridge = CvBridge() 
        msg = bridge.cv2_to_compressed_imgmsg(self.frame)
        #pub.publish(msg)
        self.image_pub.publish(msg)


####################################"


    def run(self):

        if not self.video.frame_available():
            # continue
            print('wait frame...') # pour remplacer le continue ?

        else: 
            

            self.frame = self.video.frame()
            self.display_img = self.frame.copy()
            
            self.publisher_image() # image pour le rosbag
            

            if self.Affichage == 1:
            
                    
                # redimensionner image pour l'affichage
                self.display_img = cv2.resize(self.display_img, (int(self.display_length_video), int(self.frame_size[1]/self.frame_size[0]*self.display_length_video)  ))
                
                cv2.imshow("Camera "+self.ROV_name+" "+str(self.num_cam), self.display_img)
                cv2.waitKey(1) & 0xFF
                
                



def main(args=None):


    rclpy.init(args=args)


    node_tracker = Tracker()
    
    rclpy.spin(node_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_tracker.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
    
    
