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
        Sets up class attributes `error`, `real_corners`, `robot_id`, `com_p_offset`,
        and `fov`. It also defines these attributes as numpy arrays, but does not
        provide any explicit initial values.

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
        Calculates the cosines of the angles between the vertices of a possible
        rectangle and checks if all the cosines are less than 0.1, indicating that
        the input points form a rectangle.

        Args:
            approx (list): 4 points in 2D space that define a quadrilateral, which
                is used to calculate the cosines of the angles between adjacent
                pairs of points in the quadrilateral.

        Returns:
            undefined: a boolean value indicating whether the given approximation
            is a rectangle or not.

        """
        if len(approx) != 4:
            return False

        def angle(pt1, pt2, pt0):
            """
            Calculates the angle between two points in 2D space based on the
            difference in their x and y coordinates, and returns the result as a
            decimal value between 0 and 1.

            Args:
                pt1 (int): 2D coordinate of a point in the graphics scene.
                pt2 (2-element NumPy array.): 2D coordinate of a second point that
                    is used to compute the distance between two points.
                    
                    	* `pt2`: A 2D NumPy array containing the other point in space,
                    to be used in computing the angle between the two points.
                    	* `dx1`: The difference in x-coordinates of the two points,
                    calculated as `pt1[0][0] - pt0[0][0]`.
                    	* `dy1`: The difference in y-coordinates of the two points,
                    calculated as `pt1[0][1] - pt0[0][1]`.
                    	* `dx2`: The difference in x-coordinates of the other point
                    in space, calculated as `pt2[0][0] - pt0[0][0]`.
                    	* `dy2`: The difference in y-coordinates of the other point
                    in space, calculated as `pt2[0][1] - pt0[0][1]`.
                pt0 (float): 2D coordinate of the reference point from which to
                    calculate the distance between two points.

            Returns:
                undefined: the angle between two given points, calculated as the
                hypotenuse of the vector difference divided by the magnitude of
                the vectors.

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
        1) converts an input image to HSV color space, 2) defines a range for green
        color, 3) segments green colors using inRange and erosion operations, 4)
        finds contours in the mask image, and 5) returns the coordinates of the
        detected green corners.

        Args:
            image (image, represented by a NumPy array.): 2D image to be converted
                to HSV color space and segmented for green color components.
                
                	* `image`: A numpy array of shape `(height, width, 3)`, representing
                the input image in BGR (Blue, Green, Red) colorspace.
                	* `hsv_image`: A new numpy array of shape `(height, width, 3)`,
                obtained by converting the input image to the HSV colorspace using
                `cv2.cvtColor`. The colorspace representation is based on the
                Hue-Saturation-Value (HSV) model.
                	* `lower_green`: A numpy array of shape `(3)`, representing the
                lower bounds of the green color range in the HSV colorspace. The
                elements are set to `(40, 40, 40)`.
                	* `upper_green`: A numpy array of shape `(3)`, representing the
                upper bounds of the green color range in the HSV colorspace. The
                elements are set to `(80, 255, 255)`.
                	* `mask`: A new numpy array of shape `(height, width, 3)`, obtained
                by applying the `inRange` function to `hsv_image` within the green
                color range defined by `lower_green` and `upper_green`. The resulting
                array represents the pixel values within the green range.
                	* `contours`: A list of numpy arrays of shape `(length, 2)`,
                representing the contours detected in the `mask` image using the
                `findContours` function. Each contour is represented as a list of
                two numbers, the x and y coordinates of the contour points.
                	* `corners`: A list of numpy arrays of shape `(2,)` representing
                the corners of the contours detected in the `mask` image. Each
                corner is represented as a pair of integers, the x and y coordinates
                of the contour point.

        Returns:
            undefined: a list of corner points (x, y coordinates) of the green
            regions in an image.

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
        Performs rectangle detection and matching using the Real Image and Virtual
        Corners. It returns the error between the virtual corners and the real corners.

        Args:
            params (list): 3D camera parameters (including focal length, principal
                point, and orientation) and other related values, which are used
                to compute the corners of the real image and compare them with
                those obtained from the virtual camera.

        Returns:
            undefined: the sum of the distances between the predicted virtual
            corners and the real corners.

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
        Detects green corners from a given image using the SURF algorithm, matches
        them with predefined real corners, and calculates the error between the
        two sets. It returns the error value.

        Args:
            params (ndarray.): 7-element tuple of camera parameters (3 for focus,
                4 for orientation) and the fov value, which are passed from the
                caller to initialize and modify the camera model's parameters
                during the object detection process.
                
                	* `com_p`: An array of length 7, containing the position of the
                blue corner in pixels.
                	* `com_o`: An array of length 7, containing the orientation of
                the green rectangle in degrees.
                	* `fov`: A scalar, representing the field of view of the camera
                in degrees.
                
                	Note that `params` is deserialized from a JSON dictionary, so its
                properties can be accessed using dot notation.

        Returns:
            undefined: a sum of errors between detected virtual corners and actual
            real corners.

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
        Compares a set of real-world corner points with a set of virtual corners
        detected using a green circle detector, calculates the error between the
        two sets, and returns the sum of the errors.

        Args:
            params (ndarray or NumPy matrix, which is expected to contain the
                parameters for the function's computation.): 4x4 homography matrix
                that maps the 3D points from the left camera to the right camera,
                and it is used to compute the view matrix for the right camera.
                
                	1/ `camera_translation`: This is an array containing the translation
                values for the camera.
                	2/ `camera_rotation_matrix`: This is a 3x3 array containing the
                rotation matrix values for the camera.
                	3/ `com_o_matrix`: This is an array containing the quaternion
                matrix values for the object's orientation.
                	4/ `view_matrix`: This is a 4x4 array containing the view
                transformation matrix values for the current frame.
                	5/ `width` and `height`: These are the image dimensions (width
                and height, respectively) in pixels.
                	6/ `projection_matrix`: This is a 3x4 array containing the
                projection matrix values for the current frame.
                	7/ `real_corners`: This is an array of length `n` containing the
                real-world corner coordinates for the image.
                	8/ `virtual_corners`: This is an array of length `n` containing
                the virtual corner coordinates for the image, calculated using the
                intrinsic and extrinsic parameters.
                	9/ `matched_set2`: This is a list of length `n` containing the
                indices of the matched virtual corners with real corners.
                	10/ `used_indices`: This is a set of indices representing the
                indices that have already been used in the matching process.

        Returns:
            undefined: a single integer value representing the total error between
            the predicted and real corner locations.

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
        1) reads the input images (PyBullet and Kinova), 2) detects rectangles
        using a custom implementation, 3) finds matching corners between the two
        images, 4) computes the error between real and virtual corners, and 5)
        displays the result.

        Args:
            imageKinova (8-bit depth image.): 3D reconstruction of the environment
                obtained from the depth sensor Kinova, which is used to detect and
                align the virtual corners with the real ones in the image.
                
                	* `imageKinova`: This is the output image from the Kinova camera,
                which is used to detect the corners of the rectangle. It is a
                3-dimensional numpy array with shape `(height, width, depth)`,
                where `height`, `width`, and `depth` represent the number of pixels
                in each dimension.
                	* `camera_matrices`: This is a list of 3x3 homogeneous matrices
                representing the camera's intrinsic parameters. These matrices are
                used to transform the image from the world coordinates to the image
                coordinates.
                	* `camera_positions`: This is a list of 4-vectors representing
                the camera's positions in the world coordinates. These vectors are
                used to calculate the camera's field of view (FoV).
                	* `com_p_init`, `com_p_offset`, `com_o_init`, and `com_o_offset`:
                These are the initial values for the pitch and roll angles of the
                camera, as well as their offsets. These offsets are used to correct
                the detected corners of the rectangle.
                	* `fov`: This is a scalar representing the field of view of the
                camera in degrees. It is used to calculate the size of the rectangular
                image.
                	* `real_corners`: This is a list of 4-vectors representing the
                real corners of the rectangle in the world coordinates. These
                corners are used as input for the calibration algorithm.
                
                	In the `minimize` function, the `error_function` is explained below:
                
                	* `error_function`: This is a function that takes two inputs: the
                initial values for the pitch and roll angles of the camera
                (`com_p_init`, `com_p_offset`), and the offset for the detected
                corners of the rectangle (`com_o_init`, `com_o_offset`). The
                function returns an integer representing the minimized value of
                the error.
                
                	The `error_function` calculates the error between the detected
                corners of the rectangle and the real corners of the rectangle in
                the world coordinates. It does this by first transforming the image
                points to the world coordinates using the camera's intrinsic
                parameters, and then calculating the distance between the detected
                corners and the real corners. The error is minimized by adjusting
                the initial values for the pitch and roll angles of the camera
                until the difference between the detected corners and the real
                corners is minimal.
                
                	In the `imagePybullet`, `blurredPybullet`, `edgesPybullet`,
                `contoursPybullet`, `approx`, `virtual_corners`, `real_corners`,
                and `errors` variables, their properties are explained in detail
                in the code comments. These variables represent various aspects
                of the image processing pipeline, such as the original image, the
                blurred image, the edges of the image, the contours of the image,
                the approximated contours, the virtual corners of the rectangle,
                and the errors between the detected corners and the real corners.
                
                	In summary, the `calibrate` function is a key part of the pipeline
                that corrects the detected corners of the rectangle using the real
                corners of the rectangle in the world coordinates, based on the
                camera's intrinsic parameters and field of view.
            robot_id (str): 3D pose of the robot (kinova or pybullet) and is used
                to compute the real corners of the robot's rectangle based on its
                visual appearance.

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
        Performs camera calibration for a Kinova robot using the Nelder-Mead
        algorithm to minimize the error between the predicted and actual corner
        positions in the images captured by the camera.

        Args:
            imageKinova (int): 3D point cloud data of the Kinova robot's environment
                and is used to detect green corners for visualization purposes.
            robot_id (int): 4-dimensional pose of the robot, which is used to
                retrieve the current state of the robot's linkages through the
                `getLinkState()` method.

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
        Calculates the fixed camera projection matrix, detects real and virtual
        corners from an input RGB image, and displays the detected corners using
        OpenCV's imshow function.

        Args:
            robot_id (float): 3D coordinate of the robot's body in the world frame,
                which is used to initialize the camera position and rotation for
                the visual servoing task.
            imageKinova (ndarray.): 3D point cloud data from the Kinova VCC-100
                robot's sensor, which is used to detect corners and create a 2D
                image of the robot's environment.
                
                	* `width`: The width of the image in pixels (1280).
                	* `height`: The height of the image in pixels (720).
                	* `view_matrix`: The view matrix of the camera, which represents
                the camera's intrinsic and extrinsic parameters.
                	* `projection_matrix`: The projection matrix of the camera, which
                represents the camera's intrinsic and extrinsic parameters.
                	* `imagePybullet`: A PyBullet image object that represents the
                image captured by the camera.
                
                	The function returns a tuple containing the final translation of
                the camera in 3D space, which is represented by the `result_position`
                attribute.

        """
        self.com_p, self.com_o, _, _, _, _ = p.getLinkState(robot_id, 9)

        self.com_p_init = self.com_p
        self.com_o_init = self.com_o

        self.real_corners = self.detect_green_corners(imageKinova)

        print("Initial pos: ", self.com_p, "Initial rotation: ", self.com_o)

        initial_params = np.concatenate((self.com_p, self.com_o))

        bounds = [
            (None, None),
            (None, None),
            (None, None),
        ]

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


    def get_kinova_instrinsic(self, imageKinova):
        pass

    def read_camera(self):
        """
        1) computes a projection and view matrix from FOV, aspect, nearplane, and
        farplane using pyscene's `computeProjectionMatrixFOV` and `computeViewMatrix`,
        respectively. 2) It rotates the camera's position, up vector, and eye
        position by computing a quaternion based on the given rotation angles. 3)
        It generates an RGB image from the view matrix and projection matrix using
        PyScence's `getCameraImage`.

        Returns:
            undefined: a rotated and projected RGB image of the scene.
            
            	* `rgb`: This is an array-like object that represents the RGB image
            captured by the camera. It has shape `(height, width, 3)` where height
            and width are the dimensions of the image, and the third axis represents
            the color channels (red, green, blue).
            	* `img`: This is a scalar value that represents the total number of
            pixels in the RGB image.

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
        cvKtoPulletP converst the K interinsic matrix as calibrated using Opencv
        and ROS to the projection matrix used in openGL and Pybullet.

        :param K:  OpenCV 3x3 camera intrinsic matrix
        :param w:  Image width
        :param h:  Image height
        :near:     The nearest objects to be included in the render
        :far:      The furthest objects to be included in the render
        :return:   4x4 projection matrix as used in openGL and pybullet
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
        cvPose2BulletView gets orientation and position as used
        in ROS-TF and opencv and coverts it to the view matrix used
        in openGL and pyBullet.

        :param q: ROS orientation expressed as quaternion [qx, qy, qz, qw]
        :param t: ROS postion expressed as [tx, ty, tz]
        :return:  4x4 view matrix as used in pybullet and openGL

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
        Takes in various camera and projection parameters and returns a 3D camera
        image represented as a RGB value. It creates a fixed camera matrix, extracts
        an eye-fixed target coordinate from the input image, computes a view matrix
        for the camera and projection matrices based on the eye-fixed target
        coordinates, computes a new view matrix with a translation and rotation
        added, applies it to the original camera image, converts the image from
        RGB to BGR format, and returns the result as an RGB value.

        Returns:
            undefined: a RGB image representing the camera view of the environment.

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
        # camera_translation = np.array([0.027060, 0.009970, 0.004706])
        camera_translation = np.array([0.0128723, 0.0150052, 0.00575831])
        camera_translation = np.array([-0.00113026, 0.0128052, 0.00575831])

        camera_rotation_matrix = np.array([
            [0.9999999999999999, 0.0, 0.0],
            [0.0, 0.9999999999999999, 0.0],
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
        #     [1, 0, 0, 0,
        #      0, 1, 0, 0,
        #      0, 0, 1, 0,
        #      0.0027147, 0.15365307, -1.35894322, 1]
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

    def prueba(self, robot_id, imageKinova):
        """
        Performs corner detection on an input image and compares it with a predefined
        set of real corners. It then matches the virtual corners with the real
        corners based on distance and stores the matched set in a variable. The
        errors between the virtual and real corners are calculated and printed to
        the console.

        Args:
            robot_id (int): 3D point of view of the robot's camera, which is used
                to retrieve the link state data from the kinova controller.
            imageKinova (ndarray or NumPy array.): 2D image of the KINOVA camera,
                which is used to detect green corners and compute the error between
                the detected corners and the expected ones based on the real-time
                data from the robot's cameras.
                
                	* `imageKinova`: A numpy array with shape `(N, 3, H, W)`, where
                N is the number of corners detected, `H` and `W` are the height
                and width of the image, respectively.
                	* `real_corners`: An NumPy array with shape `(N, 2)` containing
                the real-world corner locations in pixel coordinates.
                	* `virtual_corners`: An NumPy array with shape `(M, 2)` where M
                is the number of virtual corners detected, containing the virtual
                corner locations in pixel coordinates.
                	* `errors`: A NumPy array with shape `(M, 1)` containing the
                errors between the real and virtual corners.
                
                	In summary, `imageKinova` contains the results of detecting green
                corners in an image using the Kanade-Lucas-Tomasi (KLT) algorithm,
                while `real_corners`, `virtual_corners`, and `errors` contain
                additional information related to these detections.

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

