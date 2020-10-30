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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    comandos:
    1) python pacman.py -l tinyMaze -p SearchAgent --frameTime 0
    2) python pacman.py -l mediumMaze -p SearchAgent --frameTime 0
    3) python pacman.py -l bigMaze -z .5 -p SearchAgent --frameTime 0
    """

    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()
    frontier = util.Stack()

    if problem.isGoalState(start_state):
        return []

    explored = []
    frontier.push((start_state, []))

    while not frontier.isEmpty():
        state, action = frontier.pop()
        if state not in explored:
            explored.append(state)
            if problem.isGoalState(state):
                print problem.getCostOfActions(action)
                return action
            for st, act, cst in problem.getSuccessors(state):
                frontier.push((st, action + [act]))

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.

    comandos:
    1) python pacman.py -l tinyMaze -p SearchAgent -a fn=bfs --frameTime 0
    2) python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs --frameTime 0
    3) python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5 --frameTime 0
    """
    start_state = problem.getStartState()
    frontier = util.Queue()
    explored = []

    if problem.isGoalState(start_state):
        return []

    frontier.push((start_state, []))
    while not frontier.isEmpty():
        state, action = frontier.pop()
        if state not in explored:
            explored.append(state)
            if problem.isGoalState(state):
                return action
            for st, act, cst in problem.getSuccessors(state):
                frontier.push((st, action + [act]))


def uniformCostSearch(problem):
    """Search the node of least total cost first.

    Comando:
    1) python pacman.py -l tinyMaze -p SearchAgent -a fn=ucs --frameTime 0
    2) python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs --frameTime 0
    3) python pacman.py -l bigMaze -p SearchAgent -a fn=ucs -z .5 --frameTime 0

    """
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()
    frontier = util.PriorityQueue()
    explored = []

    if problem.isGoalState(start_state):
        return []

    frontier.push((start_state, [], 0), 0)
    while not frontier.isEmpty():
        state, action, cost = frontier.pop()
        if state not in explored:
            explored.append(state)
            if problem.isGoalState(state):
                return action
            for st, act, cst in problem.getSuccessors(state):
                frontier.push((st, action + [act], cost + cst), cost + cst)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first.

    Comando:
    1) python pacman.py -l tinyMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic --frameTime 0
    1) python pacman.py -l mediumMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic --frameTime 0
    1) python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic --frameTime 0

    """
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()
    frontier = util.PriorityQueue()
    explored = []

    if problem.isGoalState(start_state):
        return []

    frontier.push((start_state, [], 0), 0)
    while not frontier.isEmpty():
        state, action, cost = frontier.pop()
        if state not in explored:
            explored.append(state)
            if problem.isGoalState(state):
                return action
            for st, act, cst in problem.getSuccessors(state):
                frontier.push((st, action + [act], cost + cst), (cost + cst) + heuristic(st, problem))


def learningRealTimeAStar(problem, heuristic=nullHeuristic):
    """Execute a number of trials of LRTA* and return the best plan found.

    Comandos:
    1) python pacman.py -l smallMaze -p SearchAgent -a fn=lrta,heuristic=manhattanHeuristic
    2) python pacman.py -l mediumMaze -p SearchAgent -a fn=lrta,heuristic=manhattanHeuristic
    3) python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=lrta,heuristic=manhattanHeuristic

    """
    MAXTRIALS = 100
    heuristic_state = {}
    state = problem.getStartState()
    actions = []
    heuristic_state[state] = heuristic(state, problem)
    while not problem.isGoalState(state):
        successors = problem.getSuccessors(state)
        for successor in successors:
            successor_state = successor[0]
            heuristic_state[successor_state] = heuristic_state.get(successor_state) or heuristic(successor_state, problem)
        s_st, s_act, s_cst = min(successors, key=lambda x: x[2] + heuristic_state.get(x[0]))
        f_s = s_cst + heuristic_state.get(s_st)
        heuristic_state[state] = max(heuristic_state[state], f_s)
        actions.append(s_act)
        state = s_st
    return actions
    

# Abbreviations 
# *** DO NOT CHANGE THESE ***
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
lrta = learningRealTimeAStar
