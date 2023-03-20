# valueIterationAgents.py
# -----------------------
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


# valueIterationAgents.py
# -----------------------
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


import mdp, util

from learningAgents import ValueEstimationAgent
import collections

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp: mdp.MarkovDecisionProcess, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0
        self.runValueIteration()

    def runValueIteration(self):
        """
          Run the value iteration algorithm. Note that in standard
          value iteration, V_k+1(...) depends on V_k(...)'s.
        """
        "*** YOUR CODE HERE ***"
        for _ in range(self.iterations):
            new_values = {}
            for state in self.mdp.getStates():
                values_for_actions = {}
                for action in self.mdp.getPossibleActions(state):
                    values_for_actions[action] = self.computeQValueFromValues(state, action)
                if values_for_actions:
                    new_values[state] = max(values_for_actions.values())
                else:
                    new_values[state] = 0
            self.values = new_values
        # states = mdp.getStates()
        # actions = mdp.getPossibleActions(state)
        # transition_function = mdp.getTransitionStatesAndProbs(state, action)
        # reward_function =  mdp.getReward(state, action, nextState)
        # V = {s: 0 for s in states}
    
        # while True:
        #     delta = 0
        #     theta = 0.0001
        #     for s in states:
        #         Q = []
        #         for a in actions:
        #             q = 0
        #             for s_prime in states:
        #                 p = transition_func(s, a, s_prime)
        #                 r = reward_func(s, a, s_prime)
        #                 q += p * (r + self.discount * V[s_prime])
        #             Q.append(q)
            
        #         max_q = max(Q)
        #         delta = max(delta, abs(max_q - V[s]))
        #         V[s] = max_q
        #     if delta < theta:
        #         break
    
        # return V


    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]

    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        "*** YOUR CODE HERE ***"
        qValue = sum(prob * (self.mdp.getReward(state, action, next_state) + self.discount * self.values[next_state])
             for next_state, prob in self.mdp.getTransitionStatesAndProbs(state, action))
        return qValue

        
    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        "*** YOUR CODE HERE ***"
        possible_actions = self.mdp.getPossibleActions(state)
        return None if not possible_actions else util.Counter({action: self.computeQValueFromValues(state, action)
            for action in possible_actions}).argMax()

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)
