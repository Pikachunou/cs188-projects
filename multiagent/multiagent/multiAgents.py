# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent
from pacman import GameState

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState: GameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState: GameState, action): #
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]
        capsules = successorGameState.getCapsules() #added
        "*** YOUR CODE HERE ***" 
        foodDistances = [manhattanDistance(newPos, pellet) for pellet in newFood.asList()]
        nearestFoodDist = min(foodDistances) if foodDistances else 0
        nonScaredGhostDistances = [manhattanDistance(newPos, ghost.getPosition()) for ghost, scaredTime in zip(newGhostStates, newScaredTimes) if scaredTime == 0]
        nearestNonScaredGhostDist = min(nonScaredGhostDistances) if nonScaredGhostDistances else float('inf')
        scaredGhostDistances = [manhattanDistance(newPos, ghost.getPosition()) for ghost, scaredTime in zip(newGhostStates, newScaredTimes) if scaredTime > 0]
        nearestScaredGhostDist = min(scaredGhostDistances) if scaredGhostDistances else float('inf')

        score = successorGameState.getScore() - currentGameState.getScore()
        score += 1.0 / (nearestFoodDist + 1)
        score -= 1.0 / (nearestNonScaredGhostDist + 1)
        score += 1.0 / (nearestScaredGhostDist + 1) if nearestScaredGhostDist <= 3 else 0

        return score



def scoreEvaluationFunction(currentGameState: GameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        def minimax(agentIndex, depth, gameState):
            if gameState.isWin() or gameState.isLose() or depth == self.depth:
                return self.evaluationFunction(gameState), None

            if agentIndex == 0:
                return maxValue(agentIndex, depth, gameState)
            else:
                return minValue(agentIndex, depth, gameState)

        def maxValue(agentIndex, depth, gameState):
            v = float("-inf")
            legalActions = gameState.getLegalActions(agentIndex)
            bestAction = None

            for action in legalActions:
                successorState = gameState.generateSuccessor(agentIndex, action)
                successorValue, _ = minimax(1, depth, successorState)
                if successorValue > v:
                    v = successorValue
                    bestAction = action

            return v, bestAction

        def minValue(agentIndex, depth, gameState):
            v = float("inf")
            legalActions = gameState.getLegalActions(agentIndex)
            bestAction = None

            for action in legalActions:
                successorState = gameState.generateSuccessor(agentIndex, action)
                if agentIndex == gameState.getNumAgents() - 1:
                    successorValue, _ = minimax(0, depth + 1, successorState)
                else:
                    successorValue, _ = minimax(agentIndex + 1, depth, successorState)
                if successorValue < v:
                    v = successorValue
                    bestAction = action
            return v, bestAction
        _, bestAction = minimax(0, 0, gameState)
        return bestAction

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        def maxValue(gameState, depth, alpha, beta):
            if gameState.isWin() or gameState.isLose() or depth == 0:
                return self.evaluationFunction(gameState), None
            v = float("-inf")
            action = None
            for a in gameState.getLegalActions(0):
                successor = gameState.generateSuccessor(0, a)
                successor_value, _ = minValue(successor, depth, 1, alpha, beta)  
                if successor_value > v:
                    v = successor_value
                    action = a
                if v > beta:
                    return v, action
                alpha = max(alpha, v)
            return v, action

        def minValue(gameState, depth, agentIndex, alpha, beta):  
            if gameState.isWin() or gameState.isLose() or depth == 0:
                return self.evaluationFunction(gameState), None
            v = float("inf")
            action = None
            for a in gameState.getLegalActions(agentIndex):
                successor = gameState.generateSuccessor(agentIndex, a)
                if agentIndex == gameState.getNumAgents() - 1:  
                    successor_value, _ = maxValue(successor, depth - 1, alpha, beta)
                else:
                    successor_value, _ = minValue(successor, depth, agentIndex + 1, alpha, beta)
                if successor_value < v:
                    v = successor_value
                    action = a
                if v < alpha:
                    return v, action
                beta = min(beta, v)
            return v, action
        _, action = maxValue(gameState, self.depth, float("-inf"), float("inf"))
        return action
        """ def alphaBetaSearch(agentIndex, depth, gameState):

            return 

        def maxValue(gameState, a, b):
            v = float("-inf")
            for successor in gameState.generateSuccessor():
                v = max(v, value(successor, a, b))
                if v > b: 
                    return v
                a = max(a, v)
            return v

        def minValue(gamestate, a, b):
            v = float("inf")
            for successor in gameState.generateSuccessor():
                v = max(v, value(successor, a, b))
                if v < a: 
                    return v
                b = max(b, v)
            return v """



class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        _, action = self.expectimaxValue(gameState, self.depth, float("-inf"), float("inf"), 0, True)
        return action
    
    def expectimaxValue(self, gameState, depth, alpha, beta, agentIndex, isMaxPlayer):
        if gameState.isWin() or gameState.isLose() or depth == 0:
            return self.evaluationFunction(gameState), None
        if isMaxPlayer:
            v = float("-inf")
            action = None
            for a in gameState.getLegalActions(0):
                successor = gameState.generateSuccessor(0, a)
                successor_value, _ = self.expectimaxValue(successor, depth, alpha, beta, 1, False)  
                if successor_value > v:
                    v = successor_value
                    action = a
                if v > beta:
                    return v, action
                alpha = max(alpha, v)
            return v, action
        else:
            v = 0
            action = None
            p = 1.0 / len(gameState.getLegalActions(agentIndex))
            for a in gameState.getLegalActions(agentIndex):
                successor = gameState.generateSuccessor(agentIndex, a)
                if agentIndex == gameState.getNumAgents() - 1:  
                    successor_value, _ = self.expectimaxValue(successor, depth - 1, alpha, beta, 0, True)
                else:
                    successor_value, _ = self.expectimaxValue(successor, depth, alpha, beta, agentIndex + 1, False)
                v += p * successor_value
            return v, action

def betterEvaluationFunction(currentGameState: GameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: In short, I get important information and set them for use later. The main thing for calculating score will be the distance of food and ghosts, so next I found those distances. 
    Did some checking if the game ended, then went on to calculate the score.
    """
    "*** YOUR CODE HERE ***"
    # Getting States and Positions of important factors
    pacmanPos = currentGameState.getPacmanPosition()
    food = currentGameState.getFood()
    ghosts = currentGameState.getGhostStates()
    capsules = currentGameState.getCapsules()
    scaredTimes = [ghostState.scaredTimer for ghostState in ghosts]

    # Distance to the nearest food
    foodDistances = [manhattanDistance(pacmanPos, foodPos) for foodPos in food.asList()]
    if len(foodDistances) > 0:
        nearestFoodDist = min(foodDistances)
    else:
        nearestFoodDist = 0

    # Distance to the nearest ghost
    ghostDistances = [manhattanDistance(pacmanPos, ghost.getPosition()) for ghost in ghosts]
    nearestGhostDist = min(ghostDistances)

    # Checking the current game state
    if currentGameState.isWin():
        return float("inf")
    elif currentGameState.isLose():
        return float("-inf")

    # Stuff to calculate score
    score = currentGameState.getScore()
    score -= nearestFoodDist
    score += nearestGhostDist
    if scaredTimes[0] > 0:
        score += 50 / nearestGhostDist

    return score



# Abbreviation
better = betterEvaluationFunction
