# Video comentado para la obtencion de video por medio de protocolo de mavlink
#!/usr/bin/env python
"""
BlueRov video capture class
"""

import cv2  # Import OpenCV, a library for computer vision tasks
import gi  # Import GObject Introspection, a middleware layer between C libraries and language bindings
import numpy as np  # Import NumPy, a library for numerical operations

gi.require_version('Gst', '1.0')  # Specify the version of Gstreamer
from gi.repository import Gst  # Import Gstreamer


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

    def __init__(self, port=5600):
        """Initialize the Video object

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)  # Initialize the GStreamer library

        self.port = port  # Set the port for the video source
        self._frame = None  # Initialize the frame to None

        # Set the video source as a UDP source at the given port
        self.video_source = 'udpsrc port={}'.format(self.port)

        # Set the video codec to decode the incoming video stream
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'

        # Set the video decode pipeline to convert the video to raw format with BGR color channels
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'

        # Set the video sink configuration to emit signals, not sync to clock and drop old buffers when full
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None  # Initialize the video pipeline to None
        self.video_sink = None  # Initialize the video sink to None

        self.run()  # Start the video stream
    
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


if __name__ == '__main__':
    video = Video()

    while True:
        if not video.frame_available():
            continue

        frame = video.frame()
        frame = cv2.resize(frame, (640, 480))
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

