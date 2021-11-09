# myAgents.py
# ---------------
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

from game import Agent
from searchProblems import PositionSearchProblem
from game import Directions
#from pacman import GameState
import util
import time
import search
import sys

"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def createAgents(num_pacmen, agent='ClosestDotAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

class MyAgent(Agent):
    """
    Implementation of your agent.

    """

    def getAction(self,state):
        """
        Returns the next action the agent will take
        """

        "*** YOUR CODE HERE ***"
        legal = state.getLegalPacmanActions()
        current = state.getPacmanState().configuration.direction

       
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions)and legal:
            return self.actions[i]
            
        else:
            return Directions.STOP

    def initialize(self,state):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """

        "*** YOUR CODE HERE"
        if self.searchFunction == None: raise Exception("No search function provided for SearchAgent")
        starttime = time.time()
        problem = self.searchType(state) # Makes a new search problem
        self.actions  = self.searchFunction(problem) # Find a path
        self.getPamCanPosition(self.index)
        totalCost = problem.getCostOfActions(self.actions)
        #print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        #if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)


        
"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class ClosestDotAgent(Agent):

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)


        "*** YOUR CODE HERE ***"
        result = []
        visited = []

        heap = util.PriorityQueue()
        start = (problem.getStartState(), [], 0)
        heap.push(start, start[2])

        while not heap.isEmpty():
            (state, path, cost) = heap.pop()
            if problem.isGoalState(state):
                result = path
                break
            if state not in visited:
                visited.append(state)
                for currState, currPath, currCost in problem.getSuccessors(state):
                    newPath = path + [currPath]
                    newCost = cost + currCost
                    newState = (currState, newPath, newCost)
                    heap.push(newState, newCost)

        return result
        

    def getAction(self, state):
        return self.findPathToClosestDot(state)[0]

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below. The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState, agentIndex):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state


        "*** YOUR CODE HERE ***"
        comida = self.food.asList()
        dist = sys.maxsize

        #Buscar la comida más cercana
        for c in comida:
            distancia = util.manhattanDistance(c,x,y)

            if(distancia < dist):
                dist = distancia

        return dist == 0 #0 estamos sobre una comida

        

