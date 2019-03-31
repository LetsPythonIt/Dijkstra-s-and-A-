import A_Star as pfa
import CreateMap as cm
import cv2
import time
import glob

def optimum_path(grid_width, grid_height, walls, start, end):
		a = pfa.PathFinding()
		a.init_grid(grid_width, grid_height, walls, start, end)
		path, all_visited_nodes = a.solve()
		return path, a.closed, all_visited_nodes

def apply_astar(start=(0,10), robot_radius=5, clearance=0, end=(240,140)):
	# Create the Map
	map_object = cm.CreateMap()
	normal_map = map_object.create_map()# Map without any minkowski modification.
	video_frames = []# Stores each change in normal_map.
	
	# Find the Walls
	# map_image is minkowski modified image.
	walls, map_image = map_object.find_obstacles(radius_of_robot=robot_radius, clearance_desired=clearance)
	
	# Set Map Parameters
	grid_width = map_image.shape[1]#250
	grid_height = map_image.shape[0]#150

	# Set starting and ending positions.
	goal_position = (start[0], 150-start[1])# Adjusting for different coordinate systems
	start_position = (end[0], 150-end[1])# between problem statement and my code.

	# Find the most optimum path.# Just change this!!!!!
	path, visited_nodes, all_visited_nodes = optimum_path(grid_width, grid_height, walls, start_position, goal_position)

	# Visualisation
	path_nodes = len(path)
	total_nodes_visited = len(all_visited_nodes)

	# Visualising the nodes being observed - Yellow

	copy_normal_map = normal_map.copy()

	for count in range(total_nodes_visited):
		cell_address = all_visited_nodes.pop()
		(x,y) = (cell_address.x, cell_address.y)
		cv2.circle(copy_normal_map, (x,y), 1, (0,255,255), -1)
		# Saving Frames!
		name = 'temp_files_search_astar_rigid/frame' + str(count) + '.png'
		cv2.imwrite(name, copy_normal_map)
		# normal_map = cv2.resize(normal_map, None, fx= 1.5, fy= 1.5, interpolation= cv2.INTER_CUBIC)

	# A* Path found - Red
	for index in range(path_nodes):
		if robot_radius == 0:
			radius = 0
		else:
			radius = robot_radius
		cell_address = path.pop(0)
		(x, y) = (cell_address[0], cell_address[1])
		# Plotting Path with Red Points.
		copy_normal_map[y,x] = (0,0,255)
		# Saving Frames!
		count += 1
		name = 'temp_files_search_astar_rigid/frame' + str(count) + '.png'
		cv2.imwrite(name, copy_normal_map)
	return copy_normal_map# The final Path


# Instructions:
# Uncomment line 81 to test point robot
# Uncomment line 85 to test rigid robot


### Y-axis
### ^
### |
### |
### |
### |
### (0,0) -------------> x-axis

# for point robot; Modify only start and end parameters!
# Route_Found = apply_astar(start= (0,10), robot_radius=0, clearance=0, end=(240,100))


# for rigid robot; All Parameters can be modified!
Route_Found = apply_astar(start= (0,10), robot_radius=5, clearance=2, end=(240,140))


# Display the shortest route.
cv2.imshow('(Press Esc!) Path Found! Video created!', Route_Found)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Uncomment to create new video.
frames = glob.glob('temp_files_search_astar_rigid/*')
frames.sort()
out = cv2.VideoWriter('A_Star_Rigid_Robot.avi',cv2.cv.FOURCC('M','J','P','G'), 60, (250,150))
for index in range(len(frames)):
	out.write(cv2.imread(frames[index]))

out.release()
cv2.destroyAllWindows()
