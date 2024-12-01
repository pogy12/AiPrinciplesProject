import random

#this program creates a 20x20 grid of 1s (wall nodes) and 0s (traversable nodes), 
#each time the program is ran, walls are spanwed randomly on the grid and the position of the start and goal nodes is also randomnly spawned

class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = self.create_grid()
        self.start, self.goal = self.generate_start_goal()
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

def main():
    rows, cols = 20, 20 #size of grid
    grid_instance = Grid(rows, cols)
    grid_instance.display_grid()

if __name__ == "__main__":
    main()
