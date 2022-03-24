from collections import deque
import numpy as np
import heapq
"""
 Class PathSolver

"""

# Create PathSolver Class

class Frontier_PQ:
    def __init__(self, start, cost):
        self.states = {}
        self.q = []
        self.add(start, cost)

    def add(self, state, cost):
        #push the new state and cost to get there onto the heap
        heapq.heappush(self.q, (cost, state))
        self.states[state] = cost

    def pop(self):
        (cost, state) = heapq.heappop(self.q)  # get cost of getting to explored state
        self.states.pop(state)    # and remove from frontier
        return (cost, state)

    def replace(self, state, cost):
        #found a cheaper route to `state`, replacing old cost with new `cost`
        self.states[state] = cost
        for i, (oldcost, oldstate) in enumerate(self.q):
            if oldstate==state and oldcost > cost:
                self.q[i] = (cost, state)
                heapq._siftdown(self.q, 0, i) # now i is posisbly out of order; restore
        return

class PathSolver:

    # init for PathSolver Class
    def __init__(self):
        #Create PathSolver

    def path(self, previous, s):
        # `previous` is a dictionary chaining together the predecessor state that led to each state
        #
        # `s` will be None for the initial state
        #
        # otherwise, start from the last state `s` and recursively trace `previous` back to the initial state,
        # constructing a list of states visited as we go

        if s is None:
            return []
        else:
            return self.path(previous, previous[s])+[s]

    def pathcost(self, path, step_costs):
        #add up the step costs along a path, which is assumed to be a list output from the `path` function above

        cost = 0
        for s in range(len(path)-1):
            cost += step_costs[path[s]][path[s+1]]
        return cost


    def breadth_first_search(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing BFS")

        frontier = deque([start]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will

        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.popleft()
            for s2 in state_graph[s]:
                if (s2 not in previous) and (s2 not in frontier):
                    frontier.append(s2)
                    previous[s2] = s
                    if s2 == goal:
                        path_out = self.path(previous, s2)
                        if return_cost: return path_out, self.pathcost(path_out, state_graph)
                        return path_out

        # no solution
        if return_cost:
            return [], 0
        else:
            return []

    def depth_first_search(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing DFS")

        frontier = deque([start]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will

        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.pop() #pops last thing on queue bc thats the furthest expaned node
            for s2 in state_graph[s]:
                if (s2 not in previous) and (s2 not in frontier):
                    frontier.append(s2)
                    previous[s2] = s
                    if s2 == goal:
                        path_out = self.path(previous, s2)
                        if return_cost: return path_out, self.pathcost(path_out, state_graph)
                        return path_out

        # no solution
        if return_cost:
            return [], 0
        else:
            return []

    def uniform_cost_search(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing UCS")

        frontier = deque([start]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will
        explored = {}
        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.popleft()
            explored[s] = self.pathcost(self.path(previous, s),state_graph)
            for s2 in state_graph[s]:
                new_cost = explored[s]+state_graph[s][s2]
                if (s2 not in explored) and (s2 not in frontier):
                    frontier.append(s2)
                    previous[s2] = s
                    if s2 == goal:
                        path_out = self.path(previous, s2)
                        if return_cost: return path_out, self.pathcost(path_out, state_graph)
                        return path_out
                elif(new_cost < self.pathcost(self.path(previous, s2),state_graph)):
                    frontier.append(s2)
                    previous[s2] = s

        # no solution
        if return_cost:
            return [], 0
        else:
            return []

        return

    def h_euclidian(self, n: tuple, goal):
        h = np.sqrt(((goal[1]-n[1])**2)+((goal[0]-n[0])**2))
        return h

    def a_star_euclidian(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing a_star_euclidian")
        frontier = deque([start]) # nodes that still need to be explored
        explored = deque() # nodes that have been fully exhasted
        previous = {start: None} # retraces the path
        f = {start: self.h_euclidian(start, goal)} # f(n) = g(n) + h(n)
        g = {start: 0} #path cost to current node
        while frontier:
            s = None
            curF = None
            # finds what has the lowst f to be the next node to explore
            for minf in frontier:
                if s is None or f[minf] < curF:
                    curF = f[minf]
                    s = minf

            if s == goal:
                path_out = self.path(previous, s)
                if return_cost: return path_out, self.pathcost(path_out, state_graph), f
                return path_out, f

            frontier.remove(s)
            explored.append(s)
            #now updates the values on the neighbors
            for s2 in state_graph[s]:
                if s2 in explored: # already looked at node
                    continue
                new_cost = g[s]+state_graph[s][s2]

                if s2 not in frontier:
                    frontier.append(s2)
                elif new_cost > g[s2]:
                    continue # the path is worse so skip
                previous[s2] = s
                g[s2] = new_cost
                f[s2] = g[s2]+self.h_euclidian(s2, goal)

        # no solution
        if return_cost:
            return [], 0, f
        else:
            return [], f

        return

    def __h_manhattan(self, n: tuple, goal): # private function to return the heuristic for a_star_manhattan
        h = abs((goal[1]-n[1]))+abs((goal[0]-n[0]))
        return h

    def a_star_manhattan(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing a_star_manhattan")
        frontier = deque([start]) # nodes that still need to be explored
        explored = deque() # nodes that have been fully exhasted
        previous = {start: None} # retraces the path
        f = {start: self.__h_manhattan(start, goal)} # f(n) = g(n) + h(n)
        g = {start: 0} #path cost to current node
        while frontier:
            s = None
            curF = None
            # finds what has the lowst f to be the next node to explore
            for minf in frontier:
                if s is None or f[minf] < curF:
                    curF = f[minf]
                    s = minf

            if s == goal:
                path_out = self.path(previous, s)
                if return_cost: return path_out, self.pathcost(path_out, state_graph), f
                return path_out, f

            frontier.remove(s)
            explored.append(s)
            #now updates the values on the neighbors
            for s2 in state_graph[s]:
                if s2 in explored: # already looked at node
                    continue
                new_cost = g[s]+state_graph[s][s2]

                if s2 not in frontier:
                    frontier.append(s2)
                elif new_cost > g[s2]:
                    continue # the path is worse so skip
                previous[s2] = s
                g[s2] = new_cost
                f[s2] = g[s2]+self.__h_manhattan(s2, goal)

        # no solution
        if return_cost:
            return [], 0, f
        else:
            return [], f

        return
