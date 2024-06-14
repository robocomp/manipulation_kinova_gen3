import time

import cv2
import numpy as np
import pybullet as p
from scipy.optimize import minimize

import cv2 as cv
import glob

class Calibrator:
    def __init__(self):
        self.error = 0
        self.real_corners = []
        self.robot_id = 0
        self.com_p_offset = np.array([0.01134004, 0.00647021, -0.00534391])
        self.com_o_offset = np.array([0.06827892, -0.03868747, -0.00570964, 0.00533773])
        # self.fov = 32.5368
        self.fov = 52


    def is_rectangle(self, approx):
        if len(approx) != 4:
            return False

        def angle(pt1, pt2, pt0):
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

    def calibrate(self, imageKinova, robot_id):
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

    def get_kinova_instrinsic(self, imageKinova):
        pass

    def read_camera(self):
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

    def read_camera_fixed(self):
        # Define camera intrinsic parameters
        width = 1280  # image width
        height = 720  # image height
        f_in_pixels = 1298
        near = 0.01  # near clipping plane
        far = 100  # far clipping plane

        # Optical center in pixel coordinates
        optical_center_x_pixels = 620  # example x-coordinate in pixels
        optical_center_y_pixels = 238  # example y-coordinate in pixels

        # Convert pixel coordinates to normalized device coordinates (NDC)
        optical_center_x_ndc = (2 * optical_center_x_pixels / width) - 1
        optical_center_y_ndc = 1 - (2 * optical_center_y_pixels / height)

        fov = 2 * np.degrees(np.arctan(width / (2 * f_in_pixels)))

        f = 1.0 / np.tan(np.radians(fov) / 2.0)

        aspect = width / height
        projection_matrix = [
            [f / aspect, 0, 0, 0],
            [0, f, 0, 0],
            [0, 0, -1, -1],
            [0, 0, -0.02, 0]
        ]

        # Flatten the projection matrix for PyBullet
        projection_matrix = np.array(projection_matrix).flatten().tolist()
        projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, near, far)

        print("fixed proyection matrix", projection_matrix)

        # Define camera extrinsic parameters
        camera_translation = np.array([-0.027060, -0.009970, -0.004706])
        camera_rotation_matrix = np.array([
            [0.9999999999999999, 0.0, 0.0],
            [0.0, 0.9999999999999999, 0.0],
            [0.0, 0.0, 1.0]
        ])

        camera_translation += self.com_p
        com_o_matrix = p.getMatrixFromQuaternion(self.com_o)
        camera_rotation_matrix = np.array(com_o_matrix).reshape(3, 3)

        # print(camera_rotation_matrix)

        eye = -np.dot(camera_rotation_matrix.T, camera_translation)

        look_direction = np.array([0, 0, -1])
        target = eye + np.dot(camera_rotation_matrix.T, look_direction)

        # Calculate the up vector
        up_local = np.array([0, 1, 0])
        up = np.dot(camera_rotation_matrix.T, up_local)

        # eye += [1, 1, 0.0]
        #target = self.com_p + target

        p.setGravity(0, 0, 0)
        # p.setRealTimeSimulation(0)

        # print(eye, target, up)
        # Compute the view matrix

        # rot_matrix = p.getMatrixFromQuaternion(self.com_o)
        # rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # # Initial vectors
        # init_camera_vector = (0, 0, 1)  # z-axis
        # init_up_vector = (0, 1, 0)  # y-axis
        # # Rotated vectors
        # camera_vector = rot_matrix.dot(init_camera_vector)
        # up_vector = rot_matrix.dot(init_up_vector)
        # view_matrix = p.computeViewMatrix(self.com_p, self.com_p + 0.1 * camera_vector, up_vector)

        # view_matrix = p.computeViewMatrix(eye, target, up)

        view_matrix = [
            [-camera_rotation_matrix[0][0], camera_rotation_matrix[0][1], -camera_rotation_matrix[0][2], 0,
             -camera_rotation_matrix[1][0], camera_rotation_matrix[1][1], -camera_rotation_matrix[1][2], 0,
             -camera_rotation_matrix[2][0], camera_rotation_matrix[2][1], -camera_rotation_matrix[2][2], 0,
             # -camera_translation[0], camera_translation[1], -camera_translation[2], 1]
             0.0027147, 0.15365307, -1.35894322, 1]
        ]

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

        eye = p.loadURDF("/home/robolab/software/bullet3/data/sphere_small.urdf", basePosition=eye,
                                   baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57]))
        # target = p.loadURDF("/home/robolab/software/bullet3/data/sphere_small.urdf", basePosition=target,
        #                               baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57]))


        # Get the camera image
        img = p.getCameraImage(width, height, view_matrix, projection_matrix)
        rgb = img[2]
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        return rgb

    def prueba(self, robot_id):
        self.com_p, self.com_o, _, _, _, _ = p.getLinkState(robot_id, 9)
        image = self.read_camera_fixed()
        cv2.imshow('image', image)
        image2 = self.read_camera()
        cv2.imshow('image2', image2)
