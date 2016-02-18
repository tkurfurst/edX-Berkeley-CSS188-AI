import sys
import inspect
from search import *

class unitSearch(SearchProblem):
    
    def __init__(self):
        self.startState = 'Test'

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        return self.startState

p = unitSearch()
print p.startState
print p.getStartState()

p = SearchProblem()
#print p.startState
#print p.getStartState()


def raiseNotDefined():
    fileName = inspect.stack()[1][1]
    line = inspect.stack()[1][2]
    method = inspect.stack()[1][3]

    print "*** Method not implemented: %s at line %s of %s" % (method, line, fileName)
    sys.exit(1)

print inspect.stack()[0][0]

print dir(inspect)
print inspect.__doc__


def dfs(graph, start, path=[]):
  '''iterative depth first search from start'''
  frontier=[start]
  print "frontier:\t", frontier
  while frontier:
    node=frontier.pop(0)
    print "unexplored node:\t", node
    if node not in path:
      path=path+[node]
      print "path:\t", path
      children = graph[node]
      frontier=children+frontier
      print "new frontier:\t", frontier
  return path



def bfs(graph, start, path=[]):
  '''iterative breadth first search from start'''
  frontier=[start]
  print "frontier:\t", frontier
  while frontier:
    node=frontier.pop(0)
    print "unexplored node:\t", node
    if node not in path:
      path=path+[node]
      print "path:\t", path
      children = graph[node]
      frontier=frontier+children
      print "new frontier:\t", frontier
  return path

'''
   +---- A
   |   /   \
   |  B--D--C
   |   \ | /
   +---- E
'''
graph = {'A':['B','C'],'B':['D','E'],'C':['D','E'],'D':['E'],'E':['A']}

print 'iterative dfs\n', dfs(graph, 'A')
print ""
print 'iterative bfs\n', bfs(graph, 'A')


class node(object):
    def __init__(self, state=None):
        self.state = state
    
    def __repr__(self):
        return "node<%s> " %(self.state)

    def __eq__(self, other):
        return self.state == other.state 

c = node()
d = node("rat")

print c
print d

print c == d

l = [c, d]

print l


def vampire_test(x, y):
    d_multiplicands = {}
    d_products = {}
    for n in str(x):
        if n in d_multiplicands:
            d_multiplicands[n] += 1
        else:
            d_multiplicands[n] = 1
    for n in str(y):
        if n in d_multiplicands:
            d_multiplicands[n] += 1
        else:
            d_multiplicands[n] = 1
    for n in str(x*y):
        if n in d_products:
            d_products[n] += 1
        else: d_products[n] = 1
    return d_multiplicands == d_products

print vampire_test(21, 6)
print vampire_test(204, 615)
print vampire_test(30, -51)
print vampire_test(-246, -510)
print vampire_test(210, 600)

sorted(str(314))

def update(x, **entries):
    """Update a dict; or an object with slots; according to entries.
    >>> update({'a': 1}, a=10, b=20)
    {'a': 10, 'b': 20}
    >>> update(Struct(a=1), a=10, b=20)
    Struct(a=10, b=20)
    """
    if isinstance(x, dict):
        x.update(entries)   
    else:
        x.__dict__.update(entries) 
    return x 

def _abstract():
    print "*** Class or Function NOT DEFINED YET"

class Node:

    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0

    def __repr__(self):
        return "<Node: %s>" %(self.state)

    def path(self):
        x, result = self, [self]
        while x.parent:
            result.append(x.parent)
            x = x.parent
        result.reverse()
        return result

    def isgoal(self):
        return self.state == 'goal'

    def abs(self):
        _abstract()

s = Node('start')
e = Node('next', s)
n = Node('thomas', e)
print n
print n.parent
print n.abs()
print n.path()

class problem:

    def __init__(self, graph, initial, goal=None):
        self.initial = initial
        self.goal = goal
        self.current = self.initial
        self.graph = graph

    def successor(self, state):
        try:
            return self.graph[state]
        except:
            return state

    def goal_test(self, state):
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1    
    
    def value(self):
        pass 
    
    def initialstate(self):
        return self.initial


def treesearch(problem, frontier=[]):
    print "treesearch start"
    frontier.append(problem.initialstate())
    loops = 0
    while frontier and loops <= 1000:
        #print "frontier: ", frontier
        loops += 1
        node = frontier.pop()
        if problem.goal_test(node):
            return node
        else:
            frontier.extend(problem.successor(node))
    print loops
    return None

def graphsearch(problem, frontier=[]):
    print "graphsearch start"
    explored = {}
    frontier.append(problem.initialstate())
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node):
            return node
        if node not in explored:
            explored[node] = True
            if node not in frontier:
                frontier.extend(problem.successor(node))
    return None

graph = {'A':['B','C'],'B':['D','E'],'C':['D','E'],'D':['E'],'E':['A','G']}

p = problem(graph, 'E', 'G')

print p.graph
print p.initial
print p.goal
print p.current
print p.successor(p.initial)
print p.successor(p.goal)

print treesearch(p)
print graphsearch(p)





def solution(node):
    """
    Returns a list of actions, following parent pointers.
    """
    if node.parent == None:
        return []
    ls = solution(node.parent)
    ls.extend([node.action])

import random

def randseq(length, low, high):
    return [random.randint(low, high) for i in range(length)]

for i in range(500000):
    randseq(4, 0, 9)



