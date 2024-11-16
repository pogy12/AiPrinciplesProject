import random

#this program creates a 20x20 grid of 1s (wall nodes) and 0s (traversable nodes), 
#each time the program is ran, walls are spanwed randomly on the grid and the position of the start and goal nodes is also randomnly spawned

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

def main():
    rows, cols = 20, 20 #size of grid
    wall_density = 0.15 #wall density on grid (15% of grid are walls)
    distance_between_start_goal = 20
    grid_instance = Grid(rows, cols, wall_density, distance_between_start_goal)
    grid_instance.display_grid()

if __name__ == "__main__":
    main()