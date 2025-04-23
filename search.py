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
from game import Directions
from typing import List

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




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
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
    fringe_lifo = util.Stack()
    state, path_so_far, acc_cost = problem.getStartState(), [], 0
    fringe_lifo.push((state, path_so_far, acc_cost))
    visited = set()
    while not fringe_lifo.isEmpty():
        current_state, current_path, current_acc_cost = fringe_lifo.pop()
        if problem.isGoalState(current_state):
            return current_path
        if current_state not in visited:
            visited.add(current_state)
            successors = problem.getSuccessors(current_state)
            for new_state, direction, cost in successors:
                new_current_path = current_path + [direction]
                new_current_cost = current_acc_cost + cost
                fringe_lifo.push((new_state, new_current_path, new_current_cost))
    return []
    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    fringe_lifo = util.Queue()
    state, path_so_far, acc_cost = problem.getStartState(), [], 0
    fringe_lifo.push((state, path_so_far, acc_cost))
    visited = set()
    while not fringe_lifo.isEmpty():
        current_state, current_path, current_acc_cost = fringe_lifo.pop()
        if problem.isGoalState(current_state):
            return current_path
        if current_state not in visited:
            visited.add(current_state)
            possible_actions = problem.getSuccessors(current_state)
            for new_state, direction, cost in possible_actions:
                new_current_path = current_path + [direction]
                new_current_cost = current_acc_cost + cost
                fringe_lifo.push((new_state, new_current_path, new_current_cost))
    return []
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    fringe_lifo = util.PriorityQueue()
    state, path_so_far, acc_cost = problem.getStartState(), [], 0
    fringe_lifo.push((state, path_so_far, acc_cost), acc_cost)
    visited = set()
    while not fringe_lifo.isEmpty():
        current_state, current_path, current_acc_cost = fringe_lifo.pop()
        if problem.isGoalState(current_state):
            return current_path
        if current_state not in visited:
            visited.add(current_state)
            possible_actions = problem.getSuccessors(current_state)
            for new_state, direction, cost in possible_actions:
                new_current_path = current_path + [direction]
                new_current_cost = current_acc_cost + cost
                fringe_lifo.push((new_state, new_current_path, new_current_cost), new_current_cost)
    return []
    util.raiseNotDefined()

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    fringe_lifo = util.PriorityQueue()
    state, path_so_far, acc_cost = problem.getStartState(), [], 0
    h_cost = heuristic(state, problem)
    f_cost = acc_cost + h_cost
    fringe_lifo.push((state, path_so_far, acc_cost), f_cost)
    visited = {}
    while not fringe_lifo.isEmpty():
        current_state, current_path, current_acc_cost = fringe_lifo.pop()
        current_h_cost = heuristic(current_state,problem)
        current_f_cost = current_acc_cost + current_h_cost
        if problem.isGoalState(current_state):
            return current_path
        if (current_state not in visited.keys()) or (current_f_cost < visited[current_state]):
            visited[current_state] = current_f_cost
            possible_actions = problem.getSuccessors(current_state)
            for new_state, direction, cost in possible_actions:
                new_current_path = current_path + [direction]
                new_current_cost = current_acc_cost + cost
                new_h_cost = heuristic(new_state,problem)
                new_f_cost = new_current_cost + new_h_cost
                fringe_lifo.push((new_state, new_current_path, new_current_cost), new_f_cost)
    return []
    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
