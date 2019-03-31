import heapq

class Cell(object):
    def __init__(self, x, y, reachable):
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0


class PathFinding(object):
    def __init__(self, heuristic_value=10):# By default, uses A* Algorithm.
        # open list
        self.opened = []
        self.opened_for_plotting = []
        heapq.heapify(self.opened)
        heapq.heapify(self.opened_for_plotting)
        # visited cells list
        self.closed = list()
        # grid cells
        self.cells = []
        self.grid_height = None
        self.grid_width = None
        self.heuristic_value = heuristic_value# For A* its 10(say)

    def init_grid(self, width, height, walls, start, end):
        self.grid_height = height
        self.grid_width = width
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if (x, y) in walls:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(Cell(x, y, reachable))# cell object location.
        self.start = self.get_cell(*start)
        self.end = self.get_cell(*end)

    def get_heuristic(self, cell):
		return self.heuristic_value*(abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    def get_cell(self, x, y):
		return self.cells[x*self.grid_height + y]

    def get_adjacent_cells(self, cell):
		"""Returns adjacent cells to a cell in Clockwise order.
		"""
		cells = []
		condition_right = (cell.x < self.grid_width -1)# Left Neighbour exists?
		condition_down = (cell.y > 0)# Down Nieghbour exists?
		condition_left = (cell.x > 0)# Right Neighbour exists?
		condition_up = (cell.y < self.grid_height -1)# Up neigbour exists?

		if condition_up and condition_right:# Does top right neighbour exists?
			cells.append(self.get_cell(cell.x + 1, cell.y + 1))

		if condition_right:	
			cells.append(self.get_cell(cell.x + 1, cell.y))

		if condition_right and condition_down:# Does bottom right neigbour exists?
			cells.append(self.get_cell(cell.x + 1, cell.y - 1))

		if condition_down:
			cells.append(self.get_cell(cell.x, cell.y - 1))

		if condition_left and condition_down:# Does bottom left neighbour exists?
			cells.append(self.get_cell(cell.x - 1, cell.y - 1))

		if condition_left:
			cells.append(self.get_cell(cell.x - 1, cell.y))

		if condition_left and condition_up: # Does top left neighbour exists?
			cells.append(self.get_cell(cell.x - 1, cell.y + 1))

		if condition_up:
			cells.append(self.get_cell(cell.x, cell.y + 1))

		return cells

    def get_path(self):
		cell = self.end
		path = [(cell.x, cell.y)]
		while cell.parent is not self.start:
			cell = cell.parent
			path.append((cell.x, cell.y))

		path.append((self.start.x, self.start.y))
		path.reverse()
		return path

    def update_cell(self, adj, cell):

		man_distance = abs(adj.x - cell.x) + abs(adj.y - cell.y)
		if man_distance >= 2:# If it is a diagonal element; Increase its to-reach-cost
			adj.g = cell.g + 14
		else:
			adj.g = cell.g + 10
		adj.g = cell.g + 10
		adj.h = self.get_heuristic(adj)
		adj.parent = cell
		adj.f = adj.h + adj.g


    def solve(self):
        """Solve maze, find path to ending cell.
        @returns path or None if not found.
        """
        # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))
        heapq.heappush(self.opened_for_plotting, (self.start.f, self.start))# Used for Visualisation
        while len(self.opened):
            # pop cell from heap queue
            f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.append(cell)
            # if ending cell, return found path
            if cell is self.end:
                return self.get_path(), self.closed
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found
                        # for this adj cell.
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))
                        heapq.heappush(self.opened_for_plotting, (adj_cell.f, adj_cell))