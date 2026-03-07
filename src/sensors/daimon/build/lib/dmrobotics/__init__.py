import pkg_resources
from . import dmSDK
import numpy as np
import cv2
__version__ = pkg_resources.get_distribution("dmrobotics").version

import logging


class Sensor:
    def __init__(self, dev_id, KEEP_FPS_Print=False) -> None:
        self.hardware = dmSDK.DMV1(dev_id, KEEP_FPS_Print=KEEP_FPS_Print)
        logging.info(f"SDK Version: {__version__}")

    def reset(self):
        self.hardware.reset()

    def getRawImage(self):
        return self.hardware.getFrame()

    def getDeformation2D(self):
        return self.hardware.getDeformation()

    def getShear(self):
        return self.hardware.getShear()

    def disconnect(self):
        self.hardware.release()

    def getDepth(self):
        return self.hardware.getDepth()

def put_arrows_on_image(image, arrows, scale = 1.0):
    image = image.copy()

    scaled_flow = arrows * scale  # scale factor

    # Get start and end coordinates
    flow_start = np.stack(
        np.meshgrid(range(0, scaled_flow.shape[1], scaled_flow.shape[1]//15),
                    range(0, scaled_flow.shape[0], scaled_flow.shape[0]//15)), 2)
    
    flow_end = (scaled_flow[flow_start[:, :, 1], flow_start[:, :, 0], :] +
            flow_start).astype(np.int32)

    norm = np.linalg.norm(scaled_flow[flow_start[:, :, 1], flow_start[:, :,
                                                                    0], :],
                        axis=2)
    # print(norm.max(), norm.min())
    nz = np.nonzero(norm)

    norm = np.asarray(norm / (scaled_flow.shape[0]/30) * 255.0, dtype='uint8')
    color_image = cv2.applyColorMap(norm, cv2.COLORMAP_RAINBOW).astype('int')
    for i in range(len(nz[0])):
        y, x = nz[0][i], nz[1][i]
        cv2.arrowedLine(image,
                        pt1=tuple(flow_start[y, x]),
                        pt2=tuple(flow_end[y, x]),
                        color=(int(color_image[y, x,
                                            0]), int(color_image[y, x, 1]),
                            int(color_image[y, x, 2])),
                        thickness=1,
                        tipLength=.3)
    return image