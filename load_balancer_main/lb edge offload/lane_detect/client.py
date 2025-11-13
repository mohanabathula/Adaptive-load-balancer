import cv2
import requests
import numpy as np
import base64

SERVER_URL = "http://10.9.65.150:5010/forward"

# Load the image
image_path = "yellow_lane_2.jpg"
image = cv2.imread(image_path)

if image is None:
    print("Error: Unable to read image.")
else:
    _, img_encoded = cv2.imencode('.jpg', image)
    response = requests.post(SERVER_URL, files={'frame': ('image.jpg', img_encoded.tobytes(), 'image/jpeg')})

    if response.status_code == 200:
        data = response.json()

        # Decode binary image
        binary_image_data = base64.b64decode(data['binary_image'])
        binary_nparr = np.frombuffer(binary_image_data, np.uint8)
        binary_image = cv2.imdecode(binary_nparr, cv2.IMREAD_GRAYSCALE)

        # Decode result image
        result_image_data = base64.b64decode(data['result_image'])
        result_nparr = np.frombuffer(result_image_data, np.uint8)
        result_image = cv2.imdecode(result_nparr, cv2.IMREAD_COLOR)

        # Show both images
        cv2.imshow("Binary Image", binary_image)
        cv2.imshow("Result Image", result_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Print additional results (optional)
        print("Horizontal line y-coordinate:", data.get('horizontal_y'))
        print("Vertical (near) line:", data.get('vertical_near'))
        print("Vertical (far) line:", data.get('vertical_far'))
        print("Lane angle:", data.get('lane_angle'))
        print("Lane X:", data.get('lane_x'))

    else:
        print(f"Error: Server returned status code {response.status_code}")
