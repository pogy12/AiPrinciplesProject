import heapq
import random
import matplotlib.pyplot as plt

class Grid:
    def __init__(self, rows, cols, wall_density, distance_between_start_goal):
        self.rows = rows
        self.cols = cols
        self.wall_density = wall_density
        self.distance_between_start_goal = distance_between_start_goal
        self.grid = self.create_grid()
        self.start, self.goal = self.generate_start_goal()
        
    def create_grid(self): #function to create grid
        grid = [] #creates empty list for the grid
        for _ in range(self.rows): #loops 'rows' times, the value is specifed in the main() function
            row = [1 if random.random() < self.wall_density else 0 for _ in range(self.cols)] #generates a random number between 1 and 0. If number < wall_density add 1 (wall), else 0 (traversable node)
            grid.append(row) #adds row to the grid
        return grid

    def generate_start_goal(self): #function to generate random start and goal nodes with at least 20 nodes in between
        while True:
            start = random.randint(0, self.rows - 1), random.randint(0, self.cols - 1) #generates random row and col positon for start node
            goal = random.randint(0, self.rows - 1), random.randint(0, self.cols - 1) #generates random row and col positon for goal node
            if abs(start[0] - goal[0]) + abs(start[1] - goal[1]) >= self.distance_between_start_goal: #checks the vertical and horizontal distance between start - goal, if >= 'distance_between_start_goal' then accept, else generate new 
                break #leave the while loop

        #make sure start and goal nodes spawn at a 0 (clear path) not at a 1 (wall)
        self.grid[start[0]][start[1]] = 0 
        self.grid[goal[0]][goal[1]] = 0
        return start, goal

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

        ax.add_patch(plt.Rectangle((self.start[1], self.start[0]), 1, 1, color='green')) #plot a 1x1 green rectangle that represents the start node
        ax.add_patch(plt.Rectangle((self.goal[1], self.goal[0]), 1, 1, color='red')) #plot a 1x1 red rectagle that represents the goal node
        plt.pause(0.1) #giive a small timeframe for the visualisation to be updated before shown

        return fig, ax 

    def visualise_search(self, current): #function to visualise how the search algorithm visits nodes
        if current != self.start and current != self.goal: #if current != start or goal,
            self.ax.add_patch(plt.Rectangle((current[1], current[0]), 1, 1, color='lightblue')) #then plot a 1x1 light blue rectangle on that node
            plt.pause(0.05) #sets the pause between each rectangle drawn (in short: sets the speed of the search visualisation)

    def visualise_shortest_path(self, path): #visualises the shortest path found by the algorithm
        for node in path: #loops through each node in the path found by the search algorithm
            if node != self.start and node != self.goal: #if node != start or goal,
                self.ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow')) #then, plot a 1x1 yellow rectangle
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
                return path[::-1] #returns the path, from start to goal

            #list of all possible neighbour positions
            neighbours = [
                (current[0] + 1, current[1]),
                (current[0] - 1, current[1]),
                (current[0], current[1] + 1),
                (current[0], current[1] - 1) 
            ]

            for neighbour in neighbours:
                r, c = neighbour
                if 0 <= r < rows and 0 <= c < cols and self.grid[r][c] == 0:
                    if neighbour not in visited:
                        heapq.heappush(priority_queue, (cost + 1, neighbour))
                        parent_map[neighbour] = current

        plt.show()
        return None


class ProgramLoop:
    def __init__(self):
        self.rows, self.cols = 20, 20
        self.wall_density = 0.15
        self.distance_between_start_goal = 20

    def run(self):
        grid = Grid(self.rows, self.cols, self.wall_density, self.distance_between_start_goal)
        grid.display_grid() #initialise the grid

        visualisation = VisualiseProgram(grid.grid, grid.start, grid.goal) #initialise visualisation

        ucs = UniformCostSearch(grid.grid, grid.start, grid.goal, visualisation)
        path = ucs.search_algorithm() #run Uniform Cost Search algorithm

        if path:
            print(f"\nNumber of nodes in the shortest path: {len(path)}")
            print("\nPath found:")
            for node in path:
                print(node)
        else:
            print("\nNo path found.")

if __name__ == "__main__":
    ProgramLoop().run()
