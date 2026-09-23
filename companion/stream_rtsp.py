#!/usr/bin/env python3
import sys
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

"""
Zero-CPU-Cost RTSP Streamer
===========================
This script serves the camera stream over RTSP directly compatible with QGroundControl.
IMPORTANT: As requested, this pipeline does NO software re-encoding. 

If using a USB camera with native H.264:
  pipeline = "v4l2src device=/dev/video0 ! video/x-h264 ! h264parse ! rtph264pay name=pay0 pt=96"

If using Jetson NVMM (CSI camera) and Hardware Encoder (0 CPU Cost):
  pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1200,framerate=30/1 ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96"
"""

class RTSPServer:
    def __init__(self, port="8554", endpoint="/stream"):
        Gst.init(None)
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(port)
        
        # Adjust this pipeline depending on whether your camera outputs raw NVMM or native H264.
        # This is the Jetson hardware accelerated (nvv4l2h264enc) pipeline which consumes NO CPU for encoding.
        pipeline = (
             "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1200,framerate=30/1 ! "
             "nvv4l2h264enc insert-sps-pps=true bitrate=4000000 ! "
             "h264parse ! rtph264pay name=pay0 pt=96"
        )
        
        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(pipeline)
        factory.set_shared(True) # Allow multiple clients (like QGC and local vision pipeline) to tap in
        
        self.server.get_mount_points().add_factory(endpoint, factory)
        print(f"RTSP Server streaming at rtsp://<jetson-ip>:{port}{endpoint}")
        print(f"PIPELINE: {pipeline}")
        print("Hardware encoding (nvv4l2h264enc) or native bypass is used. ZERO CPU re-encoding overhead.")

    def run(self):
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            print("Stopping RTSP Server...")

if __name__ == '__main__':
    server = RTSPServer()
    server.run()
