import random

class Agent:
	def chooseAction(self, observations, possibleActions):
		return random.choice(possibleActions)
