import depthai as dai
import cv2
import numpy as np
import math
import os
import time
from datetime import timedelta

# User-defined constants
WARNING = 500  # 50cm, orange
CRITICAL = 250  # 30cm, red

image_path = '/home/pi/ssd/images'
metadata_path = '/home/pi/ssd/metadata'

# Create pipeline
pipeline = dai.Pipeline()

# Color camera
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(300, 300)
camRgb.setInterleaved(False)

# Define source - stereo depth cameras
left = pipeline.create(dai.node.MonoCamera)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)

right = pipeline.create(dai.node.MonoCamera)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Create stereo depth node
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setConfidenceThreshold(50)
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(True)

# Linking
left.out.link(stereo.left)
right.out.link(stereo.right)

# Spatial location calculator configuration
slc = pipeline.create(dai.node.SpatialLocationCalculator)
for x in range(15):
    for y in range(9):
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = 200
        config.depthThresholds.upperThreshold = 10000
        config.roi = dai.Rect(dai.Point2f((x+0.5)*0.0625, (y+0.5)*0.1), dai.Point2f((x+1.5)*0.0625, (y+1.5)*0.1))
        config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        slc.initialConfig.addROI(config)

stereo.depth.link(slc.inputDepth)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

# Create output
# slcOut = pipeline.create(dai.node.XLinkOut)
# slcOut.setStreamName('slc')
# slc.out.link(slcOut.input)

# colorOut = pipeline.create(dai.node.XLinkOut)
# colorOut.setStreamName('color')
# camRgb.video.link(colorOut.input)

sync = pipeline.create(dai.node.Sync)
sync.setSyncThreshold(timedelta(milliseconds=50))

slc.out.link(sync.inputs['slc'])
camRgb.video.link(sync.inputs['color'])

outGroup = pipeline.create(dai.node.XLinkOut)
outGroup.setStreamName("xout")

sync.out.link(outGroup.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Output queues will be used to get the color mono frames and spatial location data
    # qColor = device.getOutputQueue(name="color", maxSize=4, blocking=False)
    # qSlc = device.getOutputQueue(name="slc", maxSize=4, blocking=False)
    queue = device.getOutputQueue("xout", maxSize=10, blocking=False)

    fontType = cv2.FONT_HERSHEY_TRIPLEX
    line_filename = os.path.join(metadata_path, 'roi.txt')

    while True:
        start_time = int(time.time() * 1000)
        # inColor = qColor.get()  # Try to get a frame from the color camera
        # inSlc = qSlc.get()  # Try to get spatial location data
        msgGrp = queue.get()
        colorFrame = None
        slc_data = None
        for name, msg in msgGrp:
            if name == 'color':
                colorFrame = msg.getCvFrame()
            if name == 'slc':
                slc_data = msg.getSpatialLocations()

        if colorFrame is None:
            print("No color camera data")
        if slc_data is None:
            print("No spatial location data")

        if slc_data is not None and colorFrame is not None:
            ms_count = int(time.time() * 1000)
            line = '{0}, {1}'.format(ms_count, len(slc_data))
            for depthData in slc_data:
                roi = depthData.config.roi
                roi = roi.denormalize(width=colorFrame.shape[1], height=colorFrame.shape[0])

                xmin = int(roi.topLeft().x)
                ymin = int(roi.topLeft().y)
                xmax = int(roi.bottomRight().x)
                ymax = int(roi.bottomRight().y)

                coords = depthData.spatialCoordinates
                distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)

                if distance == 0:  # Invalid
                    continue

                # Determine color based on distance
                if distance < CRITICAL:
                    color = (0, 0, 255)  # Red
                    line += ',({0:.1f}: {1}, {2}, {3}, {4})'.format(distance, xmin, ymin, xmax, ymax)
                elif distance < WARNING:
                    color = (0, 140, 255)  # Orange
                    line += ',({0:.1f}: {1}, {2}, {3}, {4})'.format(distance, xmin, ymin, xmax, ymax)
                else:
                    continue  # Skip drawing for non-critical/non-warning distances

                # Draw rectangle and distance text on the color frame
                cv2.rectangle(colorFrame, (xmin, ymin), (xmax, ymax), color, thickness=2)
                cv2.putText(colorFrame, "{:.1f}m".format(distance / 1000), (xmin + 10, ymin + 20), fontType, 0.5, color)

            image_filename = os.path.join(image_path, 'frame-{}.jpg'.format(ms_count))
            cv2.imwrite(image_filename, colorFrame)
            with open(line_filename, 'a') as line_file:
                line_file.write(line)
                line_file.write('\n')
        delta_time = int(time.time() * 1000) - start_time
        sleep_time = 0.1 - (delta_time / 1000.0)
        time.sleep(max(0, sleep_time))
