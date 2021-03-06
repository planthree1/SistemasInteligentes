# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """
    def getGameState(self):

        util.raiseNotDefined()

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):

    #this start is probably current
    start = problem.getStartState()

    # this is being used to find the goal pos
    goalState = problem.getStartState()

    # creating a data structure to save the explored tiles, and adding the starting one right away
    explored = [start]

    # Creating a stack
    queue = util.Stack()
    queue.push((start, []))

    while not queue.isEmpty() and not problem.isGoalState(goalState):
        # creates a position and how it got there taking the info from the top of the stack
        # pos = position, actions = how it got there
        pos, actions = queue.pop()

        # marks that position as explored
        explored.append(pos)

        for neighbour in problem.getSuccessors(pos):
            coordinates = neighbour[0]
            if not coordinates in explored:
                goalState = neighbour[0]
                direction = neighbour[1]
                queue.push((coordinates, actions + [direction]))
    return actions + [direction]
    util.raiseNotDefined()
    '''
    while path[current_state][2] is not None:
        direction.insert(0, path[current_state][0])
        current_state = path[current_state][2]
        #print(path[current_state][2])
    '''

    return direction


def breadthFirstSearch(problem):

    start = problem.getStartState()

    exploredState = []
    exploredState.append(start)

    states = util.Queue()
    stateTuple = (start, [])

    states.push(stateTuple)

    while not states.isEmpty():
        state, action = states.pop()
        if problem.isGoalState(state):
            return action
        successor = problem.getSuccessors(state)
        for i in successor:
            coordinates = i[0]
            if not coordinates in exploredState:
                direction = i[1]
                exploredState.append(coordinates)
                states.push((coordinates, action + [direction]))
    return action
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    
    # Initiates the problem
    start = problem.getStartState()
    exploredState = []

    # Creates a prioty queue and push the initial state problem
    states = util.PriorityQueue()
    states.push((start, []), 0)

    # Loops untill there are no states to visit
    while not states.isEmpty():
        state, actions = states.pop()

        # Ends the function if the goal state is found
        if problem.isGoalState(state):
            return actions

        # Checks if the next State was already visited
        if state not in exploredState:
            successors = problem.getSuccessors(state)
            # Checks the sucessors to that state
            for succ in successors:
                coordinates = succ[0]
                #checks if the sucessor was already visited
                if coordinates not in exploredState:
                    directions = succ[1]
                    newActions = actions + [directions]
                    states.push((coordinates, actions + [directions]), problem.getCostOfActions(newActions))
                    # sucessor is pushed to the states priority Queue
        exploredState.append(state)
    return actions
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    print("null heuristic")
    return 0

def manhattanHeuristic(position, problem, info={}):  # Added by Dante
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goa
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def manhattanHeuristic2(position, problem, info={}):  # Added by Dante
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def aStarSearch(problem, heuristic=manhattanHeuristic2):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    # Start the problem
    start = problem.getStartState()

    # Store the visited state to avoid repeating a previously visited state
    exploredState = []

    # Creates a priority queue and push the initial state problem with heuristic
    states = util.PriorityQueue()
    states.push((start, []), heuristic(start, problem))

    # Loops until there are no states to visit
    while not states.isEmpty():

        # Get the actual state and actions (path)
        state, actions = states.pop()

        # Find the problem when state = (1, 1)
        if problem.isGoalState(state):
            return actions

        if state not in exploredState:
            successors = problem.getSuccessors(state)

            # Checks the sucessors to that state
            for succ in successors:
                coordinates = succ[0]
                if coordinates not in exploredState:
                    directions = succ[1]
                    nActions = actions + [directions]

                    # Calculating the actual node cost
                    nCost = problem.getCostOfActions(nActions) + heuristic(coordinates, problem)
                    states.push((coordinates, actions + [directions]), nCost)
        exploredState.append(state)
    return actions
    util.raiseNotDefined()



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
