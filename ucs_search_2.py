import heapq
import random
import matplotlib.pyplot as plt

class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = self.create_grid()
        self.start, self.goal = self.set_start_goal()
        self.generate_ramp()

    def create_grid(self): #function to create grid
        random.seed(42) #seed for reproducibility (keeps grid the same every time)
        grid = [[1 if random.random() < 0.15 else 0 for _ in range(self.cols)] for _ in range(self.rows)] #spawns 1 (wall) at a 0.15 chance rate and the rest is 0 (traversable node)
        return grid

    def generate_start_goal(self): #function to set the positions of start and goal nodes 
        start = (1, 1) #position of start node
        goal = (19, 19) #position of goal node
        return start, goal

    def generate_ramp(self): #function to generate ramps (cost == 2) on map
        random.seed(42) #seed for reproducibility (keeps ramp pistion the same every time)
        for r in range(self.rows): #for each node in row,
            for c in range(self.cols): #and for each node in column:
                if self.grid[r][c] == 0 and random.random() < 0.1: #if the current node == 0 and random value is below 0.1:
                    self.grid[r][c] = 2 #set current node to 2 (traversable node, but costs 2 to traversal)

    def display_grid(self): #function to print grid
        print("Start node:", self.start)
        print("Goal node:", self.goal)
        print("Grid:")
        for row in self.grid: #for each row in grid,
            print(row) #print row

class VisualiseProgram:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.fig, self.ax = self.grid_visualisation()

    def grid_visualisation(self): #function to visualise the grid
        rows, cols = len(self.grid), len(self.grid[0])
        fig, ax = plt.subplots() #fig is the canvas, ax is the the 'drawing' on the canvas
        ax.set_xlim(0, cols) #sets the limit of colums in visualisation so that it matches the 2D grid
        ax.set_ylim(0, rows) #sets the limit of rows in visualisation so that it matches the 2D grid
        ax.invert_yaxis() #invertys y-axis so that the visualisation is kept consistant with the 2D grid
        ax.set_xticks(range(cols)), ax.set_yticks(range(rows)) #sets the ticks in the range of the cols and rows so that each node is visualised properly
        ax.grid(True) #displays lines on the grid to seperate each node in the visualisation

        #draw the initial grid
        for i in range(rows): #iterate over every cell in rows
            for j in range(cols): #iterate over every cell in cols
                if self.grid[i][j] == 1: #if either one of the nodes in rows or cols == 1 (wall),
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black')) #then plot a black rectangle of size 1x1 at position (j, i)
                elif self.grid[i][j] == 2:  # if the node is a ramp,
                    ax.add_patch(plt.Rectangle((j, i), 1, 1, color='orange'))  #plot an orange rectangle to represent a ramp

        ax.add_patch(plt.Rectangle((self.start[1], self.start[0]), 1, 1, color='green')) #plot a 1x1 green rectangle that represents the start node
        ax.add_patch(plt.Rectangle((self.goal[1], self.goal[0]), 1, 1, color='red')) #plot a 1x1 red rectagle that represents the goal node
        plt.pause(0.1) #giive a small timeframe for the visualisation to be updated before shown

        return fig, ax 

    def visualise_search(self, current): #function to visualise how the search algorithm visits nodes
        if current != self.start and current != self.goal:
            if self.grid[current[0]][current[1]] == 2:
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='sienna')) #then plot a 1x1 sienna (mix of orange and blue) rectangle on that node to represent a checked ramp
            else:
                self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='lightblue')) #then plot a 1x1 light blue rectangle to represent a checked node
            plt.pause(0.05) #sets the pause between each rectangle drawn (in short: sets the speed of the search visualisation)

    def visualise_shortest_path(self, path): #visualises the shortest path found by the algorithm
        for node in path: #loops through each node in the path found by the search algorithm
            if node != self.start and node != self.goal: #if node != start or goal,
                if self.grid[node[0]][node[1]] == 2: #if current node == 2,
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='goldenrod')) #then plot a 1x1 goldenrod (mix of yellow and sienna) rectangle
                else:
                    self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow')) #then plot a 1x1 yellow rectangle
                plt.pause(0.1) #timeframe between each yellow rectangle being plotted

        plt.show() #displays the final visualisation of the grid

class UniformCostSearch:
    def __init__(self, grid, start, goal, visualisation):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.visualisation = visualisation

    def search_algorithm(self): #function to implement UCS on the grid
        rows, cols = len(self.grid), len(self.grid[0]) #retrieves the dimension of the grid
        priority_queue = [] #empty list for priority_queue
        heapq.heappush(priority_queue, (0, self.start)) #adds the start node to priority_queue with cost of 0
        visited = set() #keeps track of already visited nodes to prevent revisiting the same node twice
        parent_map = {self.start: None} #reconstructs the path from start
        cost_map = {self.start: 0} #keeps track of the total cost

        while priority_queue:
            cost, current = heapq.heappop(priority_queue) #pops the node with the lowest cost from the priority_queue

            if current in visited: #if current node has already been visited, continue
                continue

            visited.add(current) #otherwise, add the current node to the visited set
            self.visualisation.visualise_search(current) #reflect the currently visited node in the visualisation

            if current == self.goal: #if the curently visited node == goal node
                path = [] #empty list for storing the path
                while current is not None: #backtracks from goal to start
                    path.append(current)
                    current = parent_map[current]

                self.visualisation.visualise_shortest_path(path[::-1]) #visualises the shortest path
                return path[::-1] #returns the path, from start to goal by going -1 form current node

            #list of all possible neighbour positions to visit
            neighbours = [
                (current[0] + 1, current[1]),
                (current[0] - 1, current[1]),
                (current[0], current[1] + 1),
                (current[0], current[1] - 1) 
            ]

            for neighbour in neighbours:
                r, c = neighbour
                if 0 <= r < rows and 0 <= c < cols and self.grid[r][c] != 1:
                    move_cost = 1 if self.grid[r][c] == 0 else 2  # Cost is 1 for normal nodes, 2 for ramps
                    new_cost = cost + move_cost

                    if neighbour not in cost_map or new_cost < cost_map[neighbour]: 
                        cost_map[neighbour] = new_cost
                        heapq.heappush(priority_queue, (new_cost, neighbour))
                        parent_map[neighbour] = current

        plt.show()
        return None

class ProgramLoop:
    def __init__(self):
        self.rows, self.cols = 20, 20

    def run(self):
        grid = Grid(self.rows, self.cols)
        grid.display_grid() #initialise the grid

        visualisation = VisualiseProgram(grid.grid, grid.start, grid.goal) #initialise visualisation

        ucs = UniformCostSearch(grid.grid, grid.start, grid.goal, visualisation)
        path = ucs.search_algorithm() #run Uniform Cost Search algorithm

        if path:
            print(f"\nNumber of nodes in the shortest path: {len(path)}")
            #print(f"\nFinal cost of traversing form start to goal: {len()} ")
            print("\nPath found:")
            for node in path:
                print(node)
        else:
            print("\nNo path found.")

if __name__ == "__main__":
    ProgramLoop().run()
