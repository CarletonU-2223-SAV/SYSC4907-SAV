import cv2
import numpy as np
import airsim

count = 0
while True:
    client = airsim.VehicleClient()
    responses = client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
    response = responses[0]
    img_rgb = np.frombuffer(response.image_data_uint8, dtype=np.uint8).reshape(response.height, response.width, 3)
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    stop_data = cv2.CascadeClassifier('stop_sign.xml')

    found = stop_data.detectMultiScale(img_gray, scaleFactor=1.0095, minNeighbors=10, minSize=(20, 20))
    
    for (x, y, width, height) in found:
        count = count + 1
        cv2.rectangle(img_rgb, (x, y), (x + height, y + width), (0, 255, 0), 5)
        print(count)

    cv2.imshow("Stop sign detection", img_rgb)
    cv2.waitKey(1)

