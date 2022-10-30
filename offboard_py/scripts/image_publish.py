#! /usr/bin/env python3

import rospy
import math
import gi
import numpy as np
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Video():

	"""BlueRov video capture class constructor

	Attributes:
		port (int): Video UDP port
		video_codec (string): Source h264 parser
		video_decode (string): Transform YUV (12bits) to BGR (24bits)
		video_pipe (object): GStreamer top-level pipeline
		video_sink (object): Gstreamer sink element
		video_sink_conf (string): Sink configuration
		video_source (string): Udp source ip and port
	"""

	def __init__(self, port = (5600)): # To change the port for each UAV, do something like (5600 + uav_id_number)
		"""Summary

		Args:
		port (int, optional): UDP port
		"""

		Gst.init(None)
		print("AQUI")

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
		return array

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
				self.video_sink_conf
			])

		self.video_sink.connect('new-sample', self.callback)

	def callback(self, sink):
		sample = sink.emit('pull-sample')
		new_frame = self.gst_to_opencv(sample)
		self._frame = new_frame

		return Gst.FlowReturn.OK


def image_publish_node():

	rospy.init_node('image_publish_node', anonymous=True)

	global uav_id_number, uav_id
	uav_id = rospy.get_param("~uav_id")
	uav_id_number = uav_id[-1]

	image_pub = rospy.Publisher(uav_id + "/original_image_topic", Image, queue_size=10)

	rate = rospy.Rate(10)

	video = Video()
	bridge = CvBridge()

	rospy.sleep(0)

	while not rospy.is_shutdown():

		if not video.frame_available():
			rospy.logerr_throttle(5,'No video frame available!')
		else:
			rospy.loginfo_throttle(5,'Video frame available!')
			image_matrix = video.frame()
			try:
				#Already encoded in RGB converted image
				image_pub.publish(bridge.cv2_to_imgmsg(image_matrix, "rgb8"))
			except CvBridgeError as e:
				print(e)

		rate.sleep()

if __name__ == '__main__':
	try:
		image_publish_node()

	except rospy.ROSInterruptException:
		pass
