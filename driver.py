from robot import Robot

start = (0, 0)
goal = (8, 9)
robot = Robot()

print("Examples of robot finding a path using a BFS search algorithm")
for i in range(2):
    path, cost = robot.path_solver.breadth_first_search(start, goal, robot.env.edge_weights_2, return_cost=True)
    print("Path cost:", cost, "\n")
    robot.show_env_and_path(path)
    robot.refresh_env()

print("Examples of robot finding a path using a DFS search algorithm")
for i in range(2):
    path, cost = robot.path_solver.depth_first_search(start, goal, robot.env.edge_weights_2, return_cost=True)
    print("Path cost:", cost, "\n")
    robot.show_env_and_path(path)
    robot.refresh_env()

print("Examples of robot finding a path using a Uniform cost search algorithm")
for i in range(2):
    path, cost = robot.path_solver.uniform_cost_search(start, goal, robot.env.edge_weights_2, return_cost=True)
    print("Path cost:", cost, "\n")
    robot.show_env_and_path(path)
    robot.show_env()
    robot.refresh_env()

print("Examples of robot finding a path using a A* search with only euclidian movment")
for i in range(2):
    path, cost, fn = robot.path_solver.a_star_euclidian(start, goal, robot.env.edge_weights_2, return_cost=True)
    print("Path cost:", cost, "\n")
    robot.show_env_and_path(path)
    robot.show_env()
    robot.refresh_env()

print("Examples of robot finding a path using a A* search with only manhatten movment")
for i in range(2):
    path, cost, fn = robot.path_solver.a_star_manhattan(start, goal, robot.env.edge_weights_2, return_cost=True)
    print("Path cost:", cost, "\n")
    robot.show_env_and_path(path)
    robot.show_env()
    robot.refresh_env()
