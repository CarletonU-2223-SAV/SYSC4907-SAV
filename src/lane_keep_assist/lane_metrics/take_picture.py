import numpy as np
import os

from common.bridge import get_bridge, SCENE_IMAGE, SEGMENTATION_IMAGE


# Just a fn to take to a picture and save it while using airsim
def take_picture(index):
    sim = get_bridge()

    response = sim.get_image(1, SCENE_IMAGE)

    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    sim.write_image(os.path.normpath('o_' + index + '.png'), img_rgb)
    sim.write_image(os.path.normpath('a_' + index + '.png'), img_rgb)

    response = sim.get_image(1, SEGMENTATION_IMAGE)

    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    sim.write_image(os.path.normpath('s_' + index + '.png'), img_rgb)
