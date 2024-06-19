import time

import cv2
import numpy as np
import pybullet as p
from scipy.optimize import minimize
from pyquaternion import Quaternion

import cv2 as cv
import glob

class Calibrator:
    def __init__(self):
        """
        Initializes the `self` instance with various attributes related to the
        calibration of a robotic arm, including `error`, `real_corners`, `robot_id`,
        and offsets for the camera and the arm's position.

        """
        self.error = 0
        self.real_corners = []
        self.robot_id = 0
        # self.com_p_offset = np.array([0.01134004, 0.00647021, -0.00534391])
        self.com_p_offset = np.array([-0.027060, -0.009970, -0.004706])
        # self.com_o_offset = np.array([0.06827892, -0.03868747, -0.00570964, 0.00533773])
        self.com_o_offset = np.array([0.0, 0.0, 0.0, 0.0])
        # self.fov = 32.5368
        self.fov = 52


    def is_rectangle(self, approx):
        """
        Takes a list of four points and calculates the cosine of the angle between
        two pairs of adjacent points, sorting the values and checking if all are
        below a threshold (0.1). It returns True if all cosines are below the
        threshold, or False otherwise.

        Args:
            approx (4-dimensional numpy array.): 4-point approximation of the angle
                between pairs of points in the input list, which is used to calculate
                the cosine of the angle for further analysis in the function.
                
                	* `len(approx)` must be 4, indicating that `approx` is a list
                with 4 elements.
                	* Each element in `approx` has two items (a tuple), representing
                the coordinates of a point in the rectangle.
                
                	Therefore, the `angle` function is applied to each pair of points
                in `approx`, and the resulting cosine values are stored in the
                `cosines` list. Finally, the input is verified by checking if all
                the cosine values are less than 0.1, indicating that the rectangle
                is a valid one.

        Returns:
            undefined: a boolean value indicating whether the given point set forms
            a rectangle or not.

        """
        if len(approx) != 4:
            return False

        def angle(pt1, pt2, pt0):
            """
            Calculates the angle between two points in 2D space based on the
            difference in x and y coordinates, then returns the angle as a float
            value between 0 and π.

            Args:
                pt1 (int): 2D point located at (`dx1`, `dy1`) relative to the origin.
                pt2 (int): 2D coordinates of a point that is used to calculate the
                    distance between the two points.
                pt0 (int): 2D coordinate of the reference point used for computing
                    the distance between two points.

            Returns:
                undefined: the angle between two points in a 2D space, calculated
                as the dot product of their difference vectors divided by the
                square root of the sum of their squared distances from the origin.

            """
            dx1 = pt1[0][0] - pt0[0][0]
            dy1 = pt1[0][1] - pt0[0][1]
            dx2 = pt2[0][0] - pt0[0][0]
            dy2 = pt2[0][1] - pt0[0][1]
            return (dx1 * dx2 + dy1 * dy2) / np.sqrt((dx1 ** 2 + dy1 ** 2) * (dx2 ** 2 + dy2 ** 2))

        cosines = []
        for i in range(4):
            cosine = angle(approx[i], approx[(i + 2) % 4], approx[(i + 1) % 4])
            cosines.append(cosine)

        cosines = np.sort(np.abs(cosines))
        if all(cosine < 0.1 for cosine in cosines):
            return True
        return False

    def detect_green_corners(self, image):
        # Convertimos la imagen a espacio de color HSV
        """
        1) converts an image to HSV color space, 2) defines a range for green
        color, 3) segments green colors using inRange() and erode() functions, 4)
        finds contours using findContours() and moments() functions, 5) iterates
        over contours and obtains the center coordinates of each contour.

        Args:
            image (ndarray object.): 8-bit grayscale image that is converted to
                HSV color space and then segmented to identify green regions, which
                are then used to find contours in the image.
                
                	* `cv2.cvtColor(image, cv2.COLOR_BGR2HSV)`: Converts the input
                image from BGR (Blue, Green, Red) to HSV (Hue, Saturation, Value)
                color space.
                	* `lower_green` and `upper_green`: Defines the range of green
                colors for segmentation, where `lower_green` is the lower endpoint
                of the range and `upper_green` is the upper endpoint.

        Returns:
            undefined: a list of coordinate pairs representing the detected green
            corners in an image.

        """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definimos el rango para el color verde
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # Segmentamos el color verde
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        mask = cv2.erode(mask, None, iterations=1)

        # cv2.imshow('mask', mask)
        # cv2.waitKey(0)

        # Encontramos los contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Iteramos sobre los contornos y obtenemos el punto central del contorno
        corners = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                corners.append((cX, cY))

        return corners

    def error_function(self, params):
        #print("params: ", params)
        """
        Takes in a 3D point cloud and an initial set of real corners as input, and
        outputs the distance between the virtual corners calculated using the
        RANSAC algorithm and the real corners, along with the error between the two.

        Args:
            params (list): 7-element tuple passed from the parent class, which
                contains information about the image, including its height, width,
                and 3D reconstruction parameters.

        Returns:
            undefined: a measure of the distance between the estimated and real
            corner points.

        """
        self.com_p = params[:3]
        self.com_o = params[3:7]
        # self.fov = params[7]

        print("com_p: ", self.com_p, "com_o: ", self.com_o, "fov: ", self.fov)

        imagePybullet = self.read_camera()
        blurredPybullet = cv2.GaussianBlur(imagePybullet, (5, 5), 0)
        edgesPybullet = cv2.Canny(blurredPybullet, 50, 150)
        contoursPybullet, _ = cv2.findContours(edgesPybullet, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        virtual_corners = []

        # Iterate through the contours to find a rectangle
        for contour in contoursPybullet:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the polygon has 4 vertices, it is likely a rectangle
            # if self.is_rectangle(approx):
            if len(approx) == 4 and cv2.contourArea(approx) > 1000:
                # Draw the contour on the original image
                cv2.drawContours(imagePybullet, [approx], -1, (0, 255, 0), 3)

                # Draw circles on the corners
                for point in approx:
                    virtual_corners.append(tuple(point[0]))

        self.com_p_offset = np.array(self.com_p) - np.array(self.com_p_init)
        self.com_o_offset = np.array(self.com_o) - np.array(self.com_o_init)

        if len(virtual_corners) != 4:
            return 1000

        matched_set2 = []
        used_indices = set()
        v_corner_aux = np.array(virtual_corners)

        for point1 in self.real_corners:
            distances = np.linalg.norm(v_corner_aux - point1, axis=1)
            sorted_indices = np.argsort(distances)

            for idx in sorted_indices:
                if idx not in used_indices:
                    matched_set2.append(virtual_corners[idx])
                    used_indices.add(idx)
                    break

        virtual_corners = matched_set2

        print("//////////////////////////////////////////////////////////////////")
        print("Real corners", self.real_corners, "virtual_corners", virtual_corners)

        # Calculate the error
        self.errors = np.linalg.norm(np.array(virtual_corners) - np.array(self.real_corners), axis=1)
        print("Errors: ", self.errors, np.sum(self.errors))

        return abs(np.sum(self.errors))

    def error_function2(self, params):
        """
        Calculates the distance between a set of real corners and their virtual
        counterparts, and returns an error value based on the average Euclidean
        distance between the two sets.

        Args:
            params (tuple): 4D tensor containing the corners' coordinates and FOV,
                which is used to compute the offset of each corner relative to its
                initial position and to calculate the error between the detected
                green corners and the real ones.

        Returns:
            undefined: a sum of distances between virtual corners and real corners.

        """
        self.com_p = params[:3]
        self.com_o = params[3:7]
        self.fov = params[7]

        self.com_p_offset = np.array(self.com_p) - np.array(self.com_p_init)
        self.com_o_offset = np.array(self.com_o) - np.array(self.com_o_init)

        print("com_p: ", self.com_p, "com_o: ", self.com_o, "fov: ", self.fov)

        imagePybullet = self.read_camera()

        # cv2.imshow('Detected Rectangle', imagePybullet)
        # cv2.waitKey(1)

        virtual_corners = self.detect_green_corners(imagePybullet)

        if len(virtual_corners) < len(self.real_corners):
            print("Error: ", 2000)
            return 2000

        matched_set2 = []
        used_indices = set()
        v_corner_aux = np.array(virtual_corners)

        for point1 in self.real_corners:
            distances = np.linalg.norm(v_corner_aux - point1, axis=1)
            sorted_indices = np.argsort(distances)

            for idx in sorted_indices:
                if idx not in used_indices:
                    matched_set2.append(virtual_corners[idx])
                    used_indices.add(idx)
                    break

        virtual_corners = matched_set2

        print("//////////////////////////////////////////////////////////////////")
        print("Real corners", self.real_corners, "virtual_corners", virtual_corners)

        # Calculate the error
        self.errors = np.linalg.norm(np.array(virtual_corners) - np.array(self.real_corners), axis=1)
        print("Errors: ", self.errors, np.sum(self.errors))

        return abs(np.sum(self.errors))

    def error_function3(self, params):
        """
        Compares real and virtual corner positions to calculate an error matrix
        between both sets, returns the average absolute error in that matrix.

        Args:
            params (ndarray or NumPy array, which can be inferred from its definition
                as a matrix product of two variables in the code snippet provided.):
                3D pose of the camera and is used to calculate the view matrix,
                translation, and rotation of the camera.
                
                	* `camera_translation`: A 3x1 numpy array representing the
                translation of the camera in the world coordinate system.
                	* `camera_rotation_matrix`: A 3x3 numpy array representing the
                rotation of the camera in the world coordinate system.
                	* `com_o_matrix`: A 3x3 numpy array representing the orientation
                of the object in the world coordinate system.
                	* `view_matrix`: A 4x4 numpy array representing the view matrix
                of the camera, which maps points in the world coordinate system
                to screen coordinates.
                	* `projection_matrix`: A 4x4 numpy array representing the projection
                matrix of the camera, which maps points in the world coordinate
                system to screen coordinates.
                	* `width` and `height`: The width and height of the image,
                respectively, which are used to calculate the corners of the green
                box.
                	* `real_corners`: A list of 2d numpy arrays representing the known
                positions of the green corner in the image.
                	* `virtual_corners`: A list of 2d numpy arrays representing the
                predicted positions of the green corner in the image, based on the
                learned parameters.
                	* `errors`: A 1d numpy array representing the difference between
                the predicted and actual positions of the green corner.

        Returns:
            undefined: the sum of the errors between the detected virtual corners
            and the ground truth real corners.

        """
        camera_translation = params

        camera_rotation_matrix = np.array([
            [0.9999999999999999, 0.0, 0.0],
            [0.0, 0.9999999999999999, 0.0],
            [0.0, 0.0, 1.0]
        ])

        camera_translation += self.com_p
        com_o_matrix = p.getMatrixFromQuaternion(self.com_o)
        camera_rotation_matrix += np.array(com_o_matrix).reshape(3, 3)

        self.view_matrix = self.cvPose2BulletView(self.com_o, camera_translation)

        img = p.getCameraImage(self.width, self.height, self.view_matrix, self.projection_matrix)
        rgb = img[2]
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        virtual_corners = self.detect_green_corners(rgb)

        if len(virtual_corners) < len(self.real_corners):
            print("Error: ", 2000)
            return 2000

        matched_set2 = []
        used_indices = set()
        v_corner_aux = np.array(virtual_corners)

        for point1 in self.real_corners:
            distances = np.linalg.norm(v_corner_aux - point1, axis=1)
            sorted_indices = np.argsort(distances)

            for idx in sorted_indices:
                if idx not in used_indices:
                    matched_set2.append(virtual_corners[idx])
                    used_indices.add(idx)
                    break

        virtual_corners = matched_set2

        print("//////////////////////////////////////////////////////////////////")
        print("Real corners", self.real_corners, "virtual_corners", virtual_corners)

        # Calculate the error
        self.errors = np.linalg.norm(np.array(virtual_corners) - np.array(self.real_corners), axis=1)
        print("Errors: ", self.errors, np.sum(self.errors))

        return abs(np.sum(self.errors))


    def calibrate(self, imageKinova, robot_id):
        """
        Performs rectangle detection on an RGB image and a depth map using the
        PyBullet library, and then compares the detected rectangles with those
        obtained from the Kinova API to determine if the pose of the robot has
        changed. It also computes the error between the real and detected corners.

        Args:
            imageKinova (8-bit unsigned char, as specified by the `cv2.imread()`
                function.): 3D point cloud data obtained from the Kinova sensor,
                which is used to detect the rectangle in the image using the
                disparity map.
                
                	1/ `imageKinova`: This is the output image from the Kinova camera.
                It is a numpy array with shape `(height, width, channels)`, where
                height and width are the dimensions of the image, and channels are
                the number of color channels (typically 3 for RGB).
                	2/ `self.real_corners`: This is a list of 4 integers representing
                the real corners of the detected rectangle in pixel coordinates.
                These corners are used to find the rotation and scaling parameters
                for the camera calibration process.
                	3/ `self.com_p`: This is the initial position of the compressed
                rectangle in pixel coordinates. It is used to initialize the
                rectangle detection algorithm.
                	4/ `self.com_o`: This is the initial offset of the compressed
                rectangle in pixel coordinates. It is used to initialize the
                rectangle detection algorithm.
                	5/ `initial_params`: This is a list of numpy arrays containing
                the initial parameters for the camera calibration process, including
                the rotation and scaling matrices, and the intrinsic and extrinsic
                parameters.
                	6/ `bounds`: This is a tuple containing the lower and upper bounds
                for each parameter in the optimization process. The bounds are
                used to constrain the optimization algorithm to search within a
                specific range for each parameter.
                	7/ `method`: This is a string indicating the optimization algorithm
                to be used. In this case, `Nelder-Mead` is used for camera
                calibration. Other optimization algorithms can also be used, such
                as gradient descent or the Levenberg-Marquardt algorithm.
                	8/ `imagePybullet`: This is the output image from the PyBullet
                rendering engine. It is a numpy array with shape `(height, width,
                3)` where height and width are the dimensions of the image, and 3
                is the number of color channels (RGBA). The image is rendered using
                the detected rectangle and its corners, and it is displayed in the
                `cv2.imshow` function.
                
                	Overall, `imageKinova` provides information about the output image
                from the Kinova camera, which is used to detect the rectangle and
                calibrate the camera. The other properties of `imageKinova` are
                explained above.
            robot_id (int): 3D point cloud of the robot and is used to perform the
                registration between the camera and the robot's point cloud.

        """
        print("Calibrating...")
        self.com_p, self.com_o, _, _, _, _ = p.getLinkState(robot_id, 9)

        self.com_p_init = self.com_p
        self.com_o_init = self.com_o

        # Apply GaussianBlur to reduce noise and improve edge detection
        blurredKinova = cv2.GaussianBlur(imageKinova, (5, 5), 0)

        # Apply edge detection using Canny
        edgesKinova = cv2.Canny(blurredKinova, 50, 150)

        # Find contours in the edged image
        contoursKinova, _ = cv2.findContours(edgesKinova, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through the contours to find a rectangle
        for contour in contoursKinova:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the polygon has 4 vertices, it is likely a rectangle
            # if self.is_rectangle(approx):
            if len(approx) == 4 and cv2.contourArea(approx) > 1000:
                # Draw the contour on the original image
                print(cv2.contourArea(approx))
                cv2.drawContours(imageKinova, [approx], -1, (0, 255, 0), 3)

                # Draw circles on the corners
                for point in approx:
                    self.real_corners.append(tuple(point[0]))
                    cv2.circle(imageKinova, tuple(point[0]), 10, (0, 0, 255), -1)

        # print(self.real_corners)

        cv2.imshow('Detected Rectangle Kinova', imageKinova)

        print(self.com_p, self.com_o, self.fov)

        # self.com_p_offset = np.array(self.com_p) - np.array(self.com_p_init)
        # self.com_o_offset = np.array(self.com_o) - np.array(self.com_o_init)

        # print("error: ", self.error_function(np.concatenate((self.com_p, self.com_o))))
        # print("error: ", self.error_function(self.fov))

        # initial_params = np.concatenate((self.com_p, self.com_o, self.fov))

        # initial_params = np.concatenate((self.com_p, self.com_o))
        # initial_params = np.concatenate((initial_params, self.fov))

        # bounds = [
        #     (0.01, 0.05),
        #     (0.0005, 0.0011),
        #     (1.33, 1.39),
        #     (None, None),
        #     (None, None),
        #     (None, None),
        #     (None, None),
        #     (None, None)
        # ]
        #
        # initial_params = []
        # initial_params[:0] = np.array(self.com_p)
        # initial_params[3:] = np.array(self.com_o)
        # # print(initial_params, len(initial_params))
        # # initial_params.append(self.fov)
        # # print(initial_params, len(initial_params))
        #
        # result_position = minimize(self.error_function, initial_params, method='Nelder-Mead')#, bounds=bounds)

        print("finals offsets: ", self.com_p_offset, self.com_o_offset)

        self.com_p = self.com_p_init + self.com_p_offset
        self.com_o = self.com_o_init + self.com_o_offset

        print("finals: ", self.com_p, self.com_o)

        imagePybullet = self.read_camera()
        blurredPybullet = cv2.GaussianBlur(imagePybullet, (5, 5), 0)
        edgesPybullet = cv2.Canny(blurredPybullet, 50, 150)
        contoursPybullet, _ = cv2.findContours(edgesPybullet, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        virtual_corners = []

        # Iterate through the contours to find a rectangle
        for contour in contoursPybullet:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the polygon has 4 vertices, it is likely a rectangle
            # if self.is_rectangle(approx):
            if len(approx) == 4 and cv2.contourArea(approx) > 1000:
                # Draw the contour on the original image
                cv2.drawContours(imagePybullet, [approx], -1, (255, 0, 255), 3)

                # Draw circles on the corners
                for point in approx:
                    cv2.circle(imagePybullet, tuple(point[0]), 10, (255, 0, 0), -1)
                    virtual_corners.append(tuple(point[0]))

                cv2.circle(imagePybullet, tuple(approx[0][0]), 10, (0, 0, 255), -1)

                cv2.drawContours(imagePybullet, [np.array(self.real_corners)], -1, (0, 255, 0), 3)
                for point in self.real_corners:
                    cv2.circle(imagePybullet, point, 10, (0, 0, 255), -1)

        if len(virtual_corners) == 4:
            matched_set2 = []
            used_indices = set()
            v_corner_aux = np.array(virtual_corners)

            for point1 in self.real_corners:
                distances = np.linalg.norm(v_corner_aux - point1, axis=1)
                sorted_indices = np.argsort(distances)

                for idx in sorted_indices:
                    if idx not in used_indices:
                        matched_set2.append(virtual_corners[idx])
                        used_indices.add(idx)
                        break

            virtual_corners = matched_set2

            print("Real corners", self.real_corners, "virtual_corners", virtual_corners)

            self.errors = np.linalg.norm(np.array(virtual_corners) - np.array(self.real_corners), axis=1)
            print("Errors: ", self.errors, np.sum(self.errors))

        # Display the result
        cv2.imshow('Detected Rectangle Pybullet', imagePybullet)
        cv2.imshow('Detected Rectangle Kinova', imageKinova)

    def calibrate2(self, imageKinova, robot_id):
        """
        Is responsible for calibrating a Kinect camera's position and orientation
        using an image from the PyBullet camera. It minimizes an error function
        to determine the optimal position and rotation, and then displays the
        detected corners in both the Kinova and PyBullet images.

        Args:
            imageKinova (image object.): 2D image captured by the Kinova camera
                and is used for detecting green corners to determine the position
                and orientation of the robot.
                
                	* `imageKinova`: A Python image class object representing the
                left camera image. It has various attributes, including `height`,
                `width`, `pixels`, and `detected_corners`. The `detected_corners`
                attribute is a list of (x, y) tuples representing the corners
                detected in the left camera image.
                
                	The function explains the properties of `imagePybullet` as well:
                
                	* `imagePybullet`: A Python image class object representing the
                right camera image. It has similar attributes as `imageKinova`,
                including `height`, `width`, `pixels`, and `detected_corners`. The
                `detected_corners` attribute is a list of (x, y) tuples representing
                the corners detected in the right camera image.
            robot_id (int): 3D pose of a specific robot in the environment, which
                is used to initialize the optimization process and evaluate the
                final solution.

        """
        self.com_p, self.com_o, _, _, _, _ = p.getLinkState(robot_id, 9)

        self.com_p_init = self.com_p
        self.com_o_init = self.com_o

        self.real_corners = self.detect_green_corners(imageKinova)

        print("Initial pos: ", self.com_p, "Initial rotation: ", self.com_o, "Initial FOV:", self.fov)

        initial_params = np.concatenate((self.com_p, self.com_o))

        bounds = [
            (None, None),
            (None, None),
            (None, None),
            (None, None),
            (None, None),
            (None, None),
            (None, None),
            (32.5, 33.5)
        ]

        initial_params = []
        initial_params[:0] = np.array(self.com_p)
        initial_params[3:] = np.array(self.com_o)
        # print(initial_params, len(initial_params))
        initial_params.append(self.fov)
        # print(initial_params, len(initial_params))

        result_position = minimize(self.error_function2, initial_params, method='Nelder-Mead', bounds=bounds)

        self.com_p = self.com_p_init + self.com_p_offset
        self.com_o = self.com_o_init + self.com_o_offset

        imagePybullet = self.read_camera()

        for corner in self.real_corners:
            cv2.circle(imageKinova, corner, 8, (255, 0, 0), -1)
            cv2.circle(imagePybullet, corner, 8, (255, 0, 0), -1)

        virtual_corners = self.detect_green_corners(imagePybullet)

        for corner in virtual_corners:
            cv2.circle(imagePybullet, corner, 8, (0, 255, 0), -1)

        cv2.imshow('Detected Corners Kinova', imageKinova)
        cv2.imshow('Detected Corners Pybullet', imagePybullet)

        print("Final pos: ", self.com_p, "Final rotation: ", self.com_o, "Final FOV:", self.fov)
        print("Finals offsets: ", self.com_p_offset, self.com_o_offset)

    def calibrate3(self, robot_id, imageKinova):
        """
        Defines camera intrinsic parameters, projects an image onto a 2D plane,
        minimizes the distance between the projected corners and the detected
        corners in the projected image, and visualizes the results on both the
        original and projected images.

        Args:
            robot_id (int): 9th element of a tuple passed to the `getLinkState()`
                method, which provides the state of a specific robot connected to
                the Kinova platform.
            imageKinova (float): 2D image captured by the Kinova camera and is
                used to detect green corners in the image.

        """
        self.com_p, self.com_o, _, _, _, _ = p.getLinkState(robot_id, 9)

        self.com_p_init = self.com_p
        self.com_o_init = self.com_o

        self.real_corners = self.detect_green_corners(imageKinova)

        # Define camera intrinsic parameters
        self.width = 1280  # image width
        self.height = 720  # image height
        f_in_pixels = 1298
        near = 0.01  # near clipping plane
        far = 100  # far clipping plane

        # Optical center in pixel coordinates
        optical_center_x_pixels = 620  # example x-coordinate in pixels
        optical_center_y_pixels = 238  # example y-coordinate in pixels

        fov = 2 * np.degrees(np.arctan(self.width / (2 * f_in_pixels)))

        k = np.array([[f_in_pixels, 0, optical_center_x_pixels],
                      [0, f_in_pixels, optical_center_y_pixels],
                      [0, 0, 1]])

        self.projection_matrix = self.cvK2BulletP(k, self.width, self.height, near, far)
        print("fixed proyection matrix", self.projection_matrix)

        # camera_translation = np.array([0.027060, 0.009970, 0.004706])
        camera_translation = np.array([0.0128723, 0.0150052, 0.00575831])

        bounds = [
            (camera_translation[0]-0.01, camera_translation[0]+0.01),
            (camera_translation[1]-0.01, camera_translation[1]+0.01),
            (camera_translation[2]-0.0005, camera_translation[2]+0.0005),
        ]

        initial_params = camera_translation

        result_position = minimize(self.error_function3, initial_params, method='Nelder-Mead', bounds=bounds)

        print("Final translation: ", result_position.x)

        img = p.getCameraImage(self.width, self.height, self.view_matrix, self.projection_matrix)
        imagePybullet = img[2]
        imagePybullet = cv2.cvtColor(imagePybullet, cv2.COLOR_RGB2BGR)

        for corner in self.real_corners:
            cv2.circle(imageKinova, corner, 8, (255, 0, 0), -1)
            cv2.circle(imagePybullet, corner, 8, (255, 0, 0), -1)

        virtual_corners = self.detect_green_corners(imagePybullet)

        for corner in virtual_corners:
            cv2.circle(imagePybullet, corner, 8, (0, 255, 0), -1)

        cv2.imshow('Detected Corners Kinova', imageKinova)
        cv2.imshow('Detected Corners Pybullet', imagePybullet)

    def read_camera(self):
        """
        1) computes a view matrix and projection matrix for a camera model based
        on a field of view (FOV), aspect ratio, near plane, far plane, and comoving
        point, 2) rotates the vectors using a quaternion, and 3) generates an RGB
        image from the view matrix and projection matrix.

        Returns:
            undefined: a rotated RGB image of the camera view.
            
            	* `img`: This is a 3-dimensional NumPy array representing the RGB
            image captured by the camera. The shape of the array is (height, width,
            3), where height and width represent the dimensions of the image, and
            the third dimension represents the color channels (RGB).
            	* `rgb`: This is a 2-dimensional NumPy array representing the RGB
            image captured by the camera. The shape of the array is (height, width),
            where height and width represent the dimensions of the image.

        """
        aspect, nearplane, farplane = 1.78, 0.01, 100
        projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, nearplane, farplane)

        print("projection_matrix: ", projection_matrix)

        rot_matrix = p.getMatrixFromQuaternion(self.com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (0, 0, 1)  # z-axis
        init_up_vector = (0, 1, 0)  # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(self.com_p, self.com_p + 0.1 * camera_vector, up_vector)

        print("view_matrix: ", np.matrix(view_matrix))
        # print("eye: ", self.com_p, "target: ", self.com_p + 0.1 * camera_vector, "up: ", up_vector)

        img = p.getCameraImage(1280, 720, view_matrix, projection_matrix)
        rgb = img[2]
        rgb = cv2.rotate(rgb, cv2.ROTATE_180)  # Get the RGB image
        # print(rgb.shape)
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return rgb
        # cv2.imshow('img', rgb)
        # cv2.waitKey(2000)

    def cvK2BulletP(self, K, w, h, near, far):
        """
        Computes a projection matrix for bullet transformation.

        Args:
            K (float): 2D perspective transformation matrix, which maps points
                from the near plane to the far plane.
            w (float): 2D screen width in meters, which is used to scale the
                x-coordinates of the projection matrix.
            h (int): vertical coordinate of the far plane, which is used in the
                computation of the projection matrix.
            near (2D vector.): 2D point at which the projection matrix is applied,
                with coordinates `(0, 0)` in the function.
                
                	* `near`: A 2D vector with length of 4 representing the near plane
                coordinates. It has components `0` and `1` representing the x- and
                y-coordinates of the near plane, respectively.
            far (float): 2D distance from the near plane of the projection, used
                to calculate the ratio of near to far distances and produce the
                correct perspective distortion.

        Returns:
            undefined: a 4D numpy array representing the projection matrix for an
            orthographic camera with near and far distances given in input.
            
            	* `projection_matrix`: A 16-element array representing the projection
            matrix in homogeneous coordinates (i.e., column vectors). The first
            four elements represent the perspective factors for the near and far
            planes, while the remaining eight elements represent the translation
            and scaling of the projection coordinate system relative to the screen
            coordinate system.
            	* `transpose`: A boolean value indicating whether the output array
            is transposed or not (in this case, it is set to `True`).
            	* `T`: The transpose operation performed on the input array before
            returning it as a list of 16 floating-point numbers.
            	* `.reshape(16)`: The reshaping operation applied to the output array
            to obtain a contiguous block of size 16.

        """
        f_x = K[0, 0]
        f_y = K[1, 1]
        c_x = K[0, 2]
        c_y = K[1, 2]
        A = (near + far) / (near - far)
        B = 2 * near * far / (near - far)

        projection_matrix = [
            [2 / w * f_x, 0, (w - 2 * c_x) / w, 0],
            [0, 2 / h * f_y, (2 * c_y - h) / h, 0],
            [0, 0, A, B],
            [0, 0, -1, 0]]
        # The transpose is needed for respecting the array structure of the OpenGL
        return np.array(projection_matrix).T.reshape(16).tolist()

    def cvPose2BulletView(self, q, t):
        """
        Takes a quaternion representing the pose of a robot in ROS-TF format and
        converts it to the corresponding view matrix in OpenGl format, using the
        pybullet library. The function first rotates the quaternion by 180 degrees
        along the X axis, then computes the inverse of the pose transformation
        matrix, and finally transposes the resulting matrix to respect the array
        structure of the OpenGl view matrix.

        Args:
            q (4D array.): 4-dimensional Quaternion object that contains the
                orientation information of the object being rendered.
                
                	* `q[3]`: The third element of `q` represents the z-rotation of
                the quaternion around the world-up axis.
                	* `q[0]`: The first element of `q` represents the x-rotation of
                the quaternion around the world-up axis.
                	* `q[1]`: The second element of `q` represents the y-rotation of
                the quaternion around the world-up axis.
                	* `q[2]`: The fourth element of `q` represents the x-position of
                the quaternion along the world-up axis.
                	* `R`: The rotation matrix representation of the quaternion `q`.
                	* `t`: An optional vector representing the position of the robot
                in global coordinates. If provided, it is concatenated with the
                rotation matrix `R` to form the 4x4 pose matrix `T`.
                	* `Tc`: The inverse of the pose from the ROS-TF format, which is
                a 4x4 matrix representing the position and orientation of the robot
                in global coordinates.
                	* `viewMatrix`: The resulting view matrix representation of the
                robot's pose in the OpenGL format, which has a shape of 16 elements.
            t (4D NumPy array.): 3D translation vector of the object being rendered,
                which is appended to the rotation matrix `R` and then used to form
                the final view matrix `viewMatrix`.
                
                	* `t`: A numpy array with shape `(3, 1)` representing the top-level
                object in the ROS-TF format.
                	* `R`: The rotation matrix representing the orientation of the
                object in world coordinates.
                	* `T`: The translation vector representing the position of the
                object in world coordinates.
                
                	Note: `t` is not destructured as it is not necessary to do so for
                this function.

        Returns:
            undefined: a 16-by-4 matrix representing the OpenSceneGraph view matrix
            for rendering 3D objects in bullet.
            
            	1/ viewMatrix - A 16x4 array representing the view matrix in OpenGL
            format, which is the product of the rotation matrix and the translation
            vector in bullet convention. The transpose symbol (T) is used to ensure
            that the array structure is maintained.
            	2/ The first three columns represent the rotational parts of the view
            matrix, while the last column represents the translation part.
            	3/ The rows represent the X, Y, Z, and W axes, respectively, in the
            OpenGL convention.
            	4/ The view matrix is used to render objects in 3D space from a
            particular viewpoint in the Bullet library.

        """
        q = Quaternion([q[3], q[0], q[1], q[2]])
        R = q.rotation_matrix

        T = np.vstack([np.hstack([R, np.array(t).reshape(3, 1)]),
                       np.array([0, 0, 0, 1])])
        # Convert opencv convention to python convention
        # By a 180 degrees rotation along X
        Tc = np.array([[1, 0, 0, 0],
                       [0, -1, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]]).reshape(4, 4)

        # pybullet pse is the inverse of the pose from the ROS-TF
        T = Tc @ np.linalg.inv(T)
        # The transpose is needed for respecting the array structure of the OpenGL
        viewMatrix = T.T.reshape(16)
        return viewMatrix

    def read_camera_fixed(self):
        # Define camera intrinsic parameters
        """
        Generates a fixed camera matrix for a given extrinsic and intrinsic
        parameters, then applies it to obtain an image from the camera's perspective.
        The image is returned as a numpy array.

        Returns:
            undefined: a 3D array representing a grayscale image of the camera's
            field of view.

        """
        width = 1280  # image width
        height = 720  # image height
        f_in_pixels = 1298
        near = 0.01  # near clipping plane
        far = 100  # far clipping plane

        # Optical center in pixel coordinates
        optical_center_x_pixels = 620  # example x-coordinate in pixels
        optical_center_y_pixels = 238  # example y-coordinate in pixels

        fov = 2 * np.degrees(np.arctan(width / (2 * f_in_pixels)))

        k = np.array([[f_in_pixels, 0, optical_center_x_pixels],
                      [0, f_in_pixels, optical_center_y_pixels],
                      [0, 0, 1]])

        projection_matrix = self.cvK2BulletP(k, width, height, near, far)

        print("fixed proyection matrix", projection_matrix)

        # Define camera extrinsic parameters
        camera_translation = np.array([0.0, 0.0, 0.0])
        # camera_translation = np.array([-0.027060, -0.009970, -0.004706])
        # camera_translation = np.array([0.0028723, 0.01576, 0.00618556])

        camera_rotation_matrix = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])

        camera_translation += self.com_p
        com_o_matrix = p.getMatrixFromQuaternion(self.com_o)
        camera_rotation_matrix += np.array(com_o_matrix).reshape(3, 3)

        view_matrix = self.cvPose2BulletView(self.com_o, camera_translation)

        # view_matrix = [
        #     [-camera_rotation_matrix[0][0], camera_rotation_matrix[0][1], -camera_rotation_matrix[0][2], 0,
        #      -camera_rotation_matrix[1][0], camera_rotation_matrix[1][1], -camera_rotation_matrix[1][2], 0,
        #      -camera_rotation_matrix[2][0], camera_rotation_matrix[2][1], -camera_rotation_matrix[2][2], 0,
        #      # -camera_translation[0], camera_translation[1], -camera_translation[2], 1]
        #      0.0027147, 0.15365307, -1.35894322, 1]
        # ]

        # view_matrix = [
        #     [1.0, 0.0, 0.0, 0.0,
        #      0.0, 1.0, 0.0, 0.0,
        #      0.0, 0.0, 1, 0,
        #      -self.com_p[0], -self.com_p[1]+0.05639, -self.com_p[2]-0.00305, 1]
        # ]

        print("fixed view matrix", np.matrix(view_matrix))
        # print("eye_fixed: ", eye, "target_fixed: ", target, "up_fixed: ", up)
        # print("com_p: ", self.com_p, "camera_translation: ", camera_translation)
        print("//////////////////////////////////////////////////////////////////")

        # Get the camera image
        img = p.getCameraImage(width, height, view_matrix, projection_matrix)
        rgb = img[2]
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        return rgb

    def cube_test(self, robot_id, imageKinova):
        """
        Is used to detect green corners on two images simultaneously, and then
        match them based on distance to create a matched set of corners between
        the two images. It also calculates the errors between the matched sets and
        prints the overall error.

        Args:
            robot_id (int): 9-bit identification number of the robot for which the
                link state information is being retrieved.
            imageKinova (ndarray (numpy array) object.): 2D image of the robot's
                environment, which is used to detect green corners and match them
                with the corresponding real corners detected in the camera image.
                
                	* `imageKinova`: This variable holds an image frame captured by
                a Kinova camera. It is a numpy array with shape (height, width,
                channels), where height and width represent the dimensions of the
                image, and channels represent the number of color channels (typically
                3 for RGB).
                	* `_`: No further information is provided about this variable as
                it is not explicitly mentioned in the code snippet.

        """
        self.com_p, self.com_o, _, _, _, _ = p.getLinkState(robot_id, 9)
        image = self.read_camera_fixed()
        # image2 = self.read_camera()
        # cv2.imshow('image2', image2)

        self.real_corners = self.detect_green_corners(imageKinova)

        for corner in self.real_corners:
            cv2.circle(imageKinova, corner, 8, (255, 0, 0), -1)
            cv2.circle(image, corner, 8, (255, 0, 0), -1)

        virtual_corners = self.detect_green_corners(image)

        for corner in virtual_corners:
            cv2.circle(image, corner, 8, (0, 255, 0), -1)

        cv2.imshow('image', image)
        cv2.imshow('imageKinova', imageKinova)

        matched_set2 = []
        used_indices = set()
        v_corner_aux = np.array(virtual_corners)

        for point1 in self.real_corners:
            distances = np.linalg.norm(v_corner_aux - point1, axis=1)
            sorted_indices = np.argsort(distances)

            for idx in sorted_indices:
                if idx not in used_indices:
                    matched_set2.append(virtual_corners[idx])
                    used_indices.add(idx)
                    break

        virtual_corners = matched_set2

        self.errors = np.linalg.norm(np.array(virtual_corners) - np.array(self.real_corners), axis=1)

        print("Error: ", abs(np.sum(self.errors)))

    def square_test(self, robot_id, imageKinova):
        """
        Detects rectangles in two images by finding contours, approximating them
        with a polygon, and checking if it has four vertices and enough area. If
        a rectangle is found, it is drawn on both images, along with the corners
        of the detected rectangle. The function also calculates the distance between
        the detected rectangle and the real corners to calculate the errors.

        Args:
            robot_id (int): 3D coordinate of the camera's position and orientation
                relative to the robot's frame, which is essential for accurately
                detecting the rectangle and real corners.
            imageKinova (int): 2D image of the robot's surroundings to be processed
                for detecting rectangles, which is passed as the first argument
                to the function.

        """
        self.com_p, self.com_o, _, _, _, _ = p.getLinkState(robot_id, 9)

        # Apply GaussianBlur to reduce noise and improve edge detection
        blurredKinova = cv2.GaussianBlur(imageKinova, (5, 5), 0)

        # Apply edge detection using Canny
        edgesKinova = cv2.Canny(blurredKinova, 50, 150)

        # Find contours in the edged image
        contoursKinova, _ = cv2.findContours(edgesKinova, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through the contours to find a rectangle
        for contour in contoursKinova:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the polygon has 4 vertices, it is likely a rectangle
            # if self.is_rectangle(approx):
            if len(approx) == 4 and cv2.contourArea(approx) > 1000:
                # Draw the contour on the original image
                # print(cv2.contourArea(approx))
                cv2.drawContours(imageKinova, [approx], -1, (0, 255, 0), 3)

                # Draw circles on the corners
                for point in approx:
                    self.real_corners.append(tuple(point[0]))
                    cv2.circle(imageKinova, tuple(point[0]), 10, (0, 0, 255), -1)

        imagePybullet = self.read_camera_fixed()

        # cv2.imshow("imageKinova", imagePybullet)
        # cv2.imshow('Detected Rectangle Kinova', imageKinova)

        blurredPybullet = cv2.GaussianBlur(imagePybullet, (5, 5), 0)
        edgesPybullet = cv2.Canny(blurredPybullet, 50, 150)
        contoursPybullet, _ = cv2.findContours(edgesPybullet, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        virtual_corners = []

        # Iterate through the contours to find a rectangle
        for contour in contoursPybullet:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the polygon has 4 vertices, it is likely a rectangle
            # if self.is_rectangle(approx):
            if len(approx) == 4 and cv2.contourArea(approx) > 1000:
                # Draw the contour on the original image
                cv2.drawContours(imagePybullet, [approx], -1, (255, 0, 255), 3)

                # Draw circles on the corners
                for point in approx:
                    cv2.circle(imagePybullet, tuple(point[0]), 10, (255, 0, 0), -1)
                    virtual_corners.append(tuple(point[0]))

                cv2.circle(imagePybullet, tuple(approx[0][0]), 10, (0, 0, 255), -1)

                cv2.drawContours(imagePybullet, [np.array(self.real_corners)], -1, (0, 255, 0), 3)
                for point in self.real_corners:
                    cv2.circle(imagePybullet, point, 10, (0, 0, 255), -1)

        if len(virtual_corners) == 4:
            matched_set2 = []
            used_indices = set()
            v_corner_aux = np.array(virtual_corners)

            for point1 in self.real_corners:
                distances = np.linalg.norm(v_corner_aux - point1, axis=1)
                sorted_indices = np.argsort(distances)

                for idx in sorted_indices:
                    if idx not in used_indices:
                        matched_set2.append(virtual_corners[idx])
                        used_indices.add(idx)
                        break

            virtual_corners = matched_set2

            print("Real corners", self.real_corners, "virtual_corners", virtual_corners)

            self.errors = np.linalg.norm(np.array(virtual_corners) - np.array(self.real_corners), axis=1)
            print("Errors: ", self.errors, np.sum(self.errors))

        # Display the result
        cv2.imshow('Detected Rectangle Pybullet', imagePybullet)
        cv2.imshow('Detected Rectangle Kinova', imageKinova)

