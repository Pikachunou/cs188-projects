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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    startState = (problem.getStartState(), [])
    visited = set()
    fringe = util.Stack()
    fringe.push(startState)
    while not fringe.isEmpty():
        node = fringe.pop()
        state, path = node[0], node[1]
        if problem.isGoalState(state):
                return path
        if state not in visited:
                visited.add(state)
                for neighbor in problem.getSuccessors(state):
                    currentState = neighbor[0]
                    currentPath = path + [neighbor[1]]
                    fringe.push((currentState, currentPath))



def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    startState = (problem.getStartState(), [])
    fringe = util.Queue()
    fringe.push(startState)
    visited = set()

    while not fringe.isEmpty():
        node = fringe.pop()
        state, path = node[0], node[1]
        if problem.isGoalState(state):
                return path
        if state not in visited:
                visited.add(state)
                for neighbor in problem.getSuccessors(state):
                    currentState = neighbor[0]
                    currentPath = path + [neighbor[1]]
                    fringe.push((currentState, currentPath))


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #changed code up because there was an error with something I couldn't figure out.

    startState = (problem.getStartState(), [])
    fringe = util.PriorityQueue()
    fringe.push(startState, 0)
    visited = set()
    #count = util.Counter()
    cost = {}
    cost[startState[0]] = 0

    while not fringe.isEmpty():
        node = fringe.pop()
        state, path = node
        if problem.isGoalState(state):
            return path
        if state not in visited:
            visited.add(state)
            for neighbor in problem.getSuccessors(state):
                currentState = neighbor[0]
                currentPath = path + [neighbor[1]]
                edgeCost = neighbor[2]
                if currentState not in cost:
                    cost[currentState] = cost[state] + edgeCost
                    fringe.push((currentState, currentPath), cost[currentState])
                else:
                    newCost = cost[state] + edgeCost
                    if newCost < cost[currentState]:
                        cost[currentState] = newCost
                        fringe.push((currentState, currentPath), cost[currentState])
                #currentState = neighbor[0]
                #currentPath = path + [neighbor[1]]
                #cost = neighbor[2]
                #count[neighbor] = count[state]
                #count[neighbor] += cost
                #fringe.push((currentState, currentPath), count[neighbor])
                #count[currentState] = cost



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    thisHeuristic = heuristic(problem.getStartState(), problem)
    startState = (problem.getStartState(), [], 0, thisHeuristic, thisHeuristic)
    fringe = util.PriorityQueue()
    fringe.push(startState, thisHeuristic)
    visited = set()

    while not fringe.isEmpty():
        node = fringe.pop()
        state, path, cost, aHeuristic, f = node[0], node[1], node[2], node[3], node[4]
        if problem.isGoalState(state):
                return path
        if state not in visited:
                visited.add(state)
                for neighbor in problem.getSuccessors(state):
                    currentState = neighbor[0]
                    currentCost = cost + neighbor[2]
                    currentPath = path + [neighbor[1]]
                    currentHeuristic = heuristic(currentState, problem)
                    f = currentCost + currentHeuristic
                    fringe.push((currentState, currentPath, currentCost, currentHeuristic, f), f)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
