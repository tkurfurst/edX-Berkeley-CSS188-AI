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

class Node:

  def __init__(self, state, action=None, step_cost=0, total_cost=0, parent=None, path=[]):
    self.state = state
    self.action = action
    self.step_cost = step_cost
    self.total_cost = total_cost
    self.parent = parent
    self.path = path

  def __repr__(self):
    return "<node: %s>" %(self.state)

  def path(self):
    
    x, result = self, [self]
    
    while x.parent:
      result.append(x.parent)
      x = x.parent
    result.reverse()  
    return result

  def expand(self, problem):
    return [Node(state, action, step_cost, self) \
      for (state, action, step_cost) in problem.getSuccessors(self.state)]

# tree_search needs to be updated to conform to graph_search below
def tree_search(problem, frontier=[]):
    state = problem.getStartState()
    path = []
    node = (state, path)
    frontier.push(node)
    while frontier:
        node = (state, path) = frontier.pop()
        if problem.isGoalState(state):
            return state
        else:
          for successor in problem.getSuccessors(state):
            frontier.push(successor)
    return None

# review PriorityQueue and PriorityQueuewithFunction
def graph_search(problem, frontier=[]):
    explored = {}
    #state = problem.getStartState()
    #path = []
    #node = (state, path)
    node = Node(problem.getStartState())
    frontier.push(node)
    while frontier:
        #node = (state, path) = frontier.pop()
        node = frontier.pop()
        if problem.isGoalState(node.state):
            return node.path
        if node.state not in explored:
            explored[node.state] = True
            for successor in problem.getSuccessors(node.state):
              (state, action, cost) = successor
              if state not in explored: #and successor not in frontier:
                #print successor in explored
                #newstate = successorstate
                #newpath = node.path + [action]
                #node = (newstate, newpath)
                newnode = Node(state, action, cost, total_cost=node.total_cost+cost, parent=node, path=node.path + [action])
                frontier.push(newnode)
    return None

"""
def dfs(problem):
  return graphsearch(problem, util.Stack())

def bfs(problem):
  return graphsearch(problem, util.Queue())
"""

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
    """
    "*** YOUR CODE HERE ***"

    #print "Start:", problem.getStartState()
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())
    
    return graph_search(problem, util.Stack())

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    return graph_search(problem, util.Queue())

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    return graph_search(problem, util.PriorityQueueWithFunction(lambda node: node.total_cost))
    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    return graph_search(problem, util.PriorityQueueWithFunction(lambda node: node.total_cost + heuristic(node.state, problem)))

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch



