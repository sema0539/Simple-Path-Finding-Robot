from enviornment import Environment
from pathSolver import PathSolver


"""
Robot Class
"""

class Robot:

    def __init__(self, seed: int = 0):
        """Create robot with an Environment and PathSolver member variables"""
        size = (10, 10)
        self.env = Environment(size, seed)
        self.path_solver = PathSolver()


    def show_env(self):
        """Displays the robot's setup call refresh_env to create a new random enviornment"""
        self.env.show_env(self.env.env)

    def show_env_and_path(self, path):
        """Displays the robot's setup with a provided path returned from a PathSolver class function"""
        self.env.show_env_and_path(path)

    def refresh_env(self):
        """Incraments the random enviornment seed and generates a new enviornment"""
        self.env = Environment(self.env.size, (self.env.seed + 1))
