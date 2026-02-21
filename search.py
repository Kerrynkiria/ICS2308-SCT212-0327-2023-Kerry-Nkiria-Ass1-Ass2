# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm 
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from util import Stack  # Stack is used to store nodes to explore next
    fringe = Stack()
    visited = set()  # Keeps track of already visited states

    # Start from the initial state with an empty path
    start_state = problem.getStartState()
    fringe.push((start_state, []))  # (current_state, path_taken)

    # Continue until there are no more nodes to explore
    while not fringe.isEmpty():
        state, path = fringe.pop()  # Take the top node from the stack

        # Check if we've reached the goal
        if problem.isGoalState(state):
            return path  # Return the path that led to the goal

        # If this state hasn’t been visited, explore it
        if state not in visited:
            visited.add(state)

            # Get all possible next moves from this state
            for successor, action, cost in problem.getSuccessors(state):
                if successor not in visited:
                    # Add the next state to the stack with the updated path
                    new_path = path + [action]
                    fringe.push((successor, new_path))

    # If no path to goal is found
    return []


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    "*** YOUR CODE HERE ***"
    from util import Queue
    fringe = Queue()
    visited = set()

    # Each element in the fringe: (state, path_to_state)
    start_state = problem.getStartState()
    fringe.push((start_state, []))
    visited.add(start_state)

    while not fringe.isEmpty():
        state, path = fringe.pop()
        if problem.isGoalState(state):
            return path
        for successor, action, stepCost in problem.getSuccessors(state):
            if successor not in visited:
                visited.add(successor)
                fringe.push((successor, path + [action]))

    return []

      
def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    fringe = PriorityQueue()
    visited = dict()

    start_state = problem.getStartState()
    fringe.push((start_state, [], 0), 0)

    while not fringe.isEmpty():
        state, path, cost_so_far = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in visited or cost_so_far < visited[state]:
            visited[state] = cost_so_far
            for successor, action, stepCost in problem.getSuccessors(state):
                new_cost = cost_so_far + stepCost
                if successor not in visited or new_cost < visited.get(successor, float('inf')):
                    fringe.push((successor, path + [action], new_cost), new_cost)

    return []


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    fringe = PriorityQueue()
    visited = dict()

    start_state = problem.getStartState()
    start_priority = heuristic(start_state, problem)
    fringe.push((start_state, [], 0), start_priority)

    while not fringe.isEmpty():
        state, path, cost_so_far = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in visited or cost_so_far < visited[state]:
            visited[state] = cost_so_far
            for successor, action, stepCost in problem.getSuccessors(state):
                new_cost = cost_so_far + stepCost
                priority = new_cost + heuristic(successor, problem)
                if successor not in visited or new_cost < visited.get(successor, float('inf')):
                    fringe.push((successor, path + [action], new_cost), priority)

    return []

    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
