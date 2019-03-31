from __future__ import division # For Float Division; Otherwise Ellipse would show fancy curves in Python2
import cv2
import numpy as np
import matplotlib.pyplot as plt


class CreateMap:
    
    def __init__(self, minkowski_required=False, robot_radius=0, clearance=0):
        self.minkowski_req = minkowski_required
        self.minkowski_dis = robot_radius + clearance
        self.obstacles = None
    
    # Creating the Circular Obstacle.
    def inside_a_circle(self, point):
        # @ point be in format: [x, y]
        x, y = point[0], point[1]
        if self.minkowski_req == True:
            radius = 15 + self.minkowski_dis
        else:
            radius = 15

        if ((x-20)**2 + (y-190)**2) <= (radius**2):
            return True
        else:
            return False
    
    # Creating the ellipse obstacle
    def inside_an_ellipse(self, point):

        x, y = point[0], point[1]

        if self.minkowski_req == True:
            min_axis = 12 + self.minkowski_dis
            maj_axis = 30 + self.minkowski_dis
        else:
            min_axis = 12
            maj_axis = 30

        if (((x-30)**2)/min_axis**2 + ((y-140)**2)/maj_axis**2) <= 1:
            return True
        else:
            return False
        
    # Creating Rectangle Equations; Half Planes.
    def inside_a_rectangle(self, point):
        x, y = point[0], point[1]

        if self.minkowski_req == True:
            offset = self.minkowski_dis
        else:
            offset = 0

        if (x<=83.5+offset) and (y<=100+offset) and (x>=33.5-offset) and (y>=50-offset):
            return True
        else:
            return False

    # Writing Polygon Equations; Half Planes.
    def inside_a_polygon(self, point):
        x, y = point[0], point[1]

        if self.minkowski_req == True:
            offset = self.minkowski_dis
        else:
            offset = 0

        h1 = (x<=135+offset)
        h2 = (y<=-0.5405*x+245.97+offset)

        h3 = (x>=98)
        h9 = (x<=98)

        h4 = (y>=-0.35135*x+197.4324)
        h10 =(y<=-0.35135*x+197.4324)

        h5 = (y<=0.60526*x+133.684+offset)
        h6 = (y>=-0.1842*x+181.0526-offset)
        h7 = (y<=9.5*x-768+offset)
        h8 = (y>=0.609756*x+67.6829-offset)


        # Convex Sets
        convex_set_1 = h1 and h2 and h3 and h4
        convex_set_2 = h5 and h6 and h9
        convex_set_3 = h10 and h7 and h8
        if convex_set_1 or convex_set_2 or convex_set_3:
            return True
        else:
            return False
    
    def create_map(self):
        self.minkowsky_req = False
        # Create a white background
        background = np.zeros((150,250,3), np.uint8)# Height, Width: Numpy nomenclature.
        background[:,:] = (255, 255, 255)# White Background
        
        # Returns a Obstacle Map without minkowsky distance.
        image_copy = background.copy()
        for x in range(150):
            for y in range(250):
                point = [x, y]
                if self.inside_a_circle(point) or self.inside_an_ellipse(point) or self.inside_a_rectangle(point) or self.inside_a_polygon(point):
                    image_copy[x,y] = (0,0,0)
                else:
                    continue
        return image_copy

    def create_map_minkowski_equations(self):
        self.minkowski_req = True
        # Create a white background
        background = np.zeros((150,250,3), np.uint8)# Height, Width: Numpy nomenclature.
        background[:,:] = (255, 255, 255)# White Background
        
        # Returns a Obstacle Map without minkowsky distance.
        map_image_copy = background.copy()
        for x in range(150):
            for y in range(250):
                point = [x, y]
                if self.inside_a_circle(point) or self.inside_an_ellipse(point) or self.inside_a_rectangle(point) or self.inside_a_polygon(point):
                    map_image_copy[x,y] = (0,0,0)
                else:
                    continue
        return map_image_copy

    def create_map_minkowski_contours(self, image):
        # @ input image of map without minkowski distance.
        # Returns a Obstacle Map without minkowski distance.
        map_image_copy = image.copy()
        edges = cv2.Canny(map_image_copy,100,200)
        contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(map_image_copy, contours, -1, (0,255,0), 1)
        contours = np.array(contours).squeeze()

        for shapes in contours:
            for contour_of_shape in shapes:
                [x,y] = contour_of_shape.squeeze()
                rad_circle = self.minkowski_dis
                cv2.circle(map_image_copy, (x,y), rad_circle, (0,0,0), -1)
        return map_image_copy



    def convert_to_grid(self, image, resolution=1):
        grid = np.zeros((image.shape[0], image.shape[1]))
        # Discretizing the Image in grids
        for x in range(image.shape[0]):
            for y in range(image.shape[1]):
                if image[x][y][1] == 0:
                    grid[x,y] = 1# grid contains 1 for obstacles.
        return grid

    def find_obstacles(self, radius_of_robot, clearance_desired, type=1):
        # type=1: equations method
        # type=2: contour method
        self.minkowski_dis = radius_of_robot + clearance_desired
        map_original = self.create_map()
        minkowski_map = self.create_map_minkowski_contours(map_original)
        grid = self.convert_to_grid(minkowski_map)
        obstacles = tuple(np.argwhere(grid == 1)) # Obstacles position        
        # Converting list into tuples for easy access.
        obstacles_tuples = []# This list will be converted into tuple of tuples.
        for obst in obstacles:
            obstacles_tuples.append((obst[1],obst[0]))# This takes care of numpy and opencv coordinate system difference as well.
        self.obstacles = tuple(obstacles_tuples)
        return self.obstacles, minkowski_map


# Store the Obstacles.
CM = CreateMap()
obstacles, image = CM.find_obstacles(radius_of_robot=5, clearance_desired=0)
image_main = CM.create_map()
image_minkowski = CM.create_map_minkowski_equations()


if __name__ == "__main__":
    cv2.imshow('minkowski', image)
    cv2.waitKey(0)
    cv2.imshow('just osbtacles', image_main)
    cv2.waitKey(0)
    cv2.destroyAllWindows()