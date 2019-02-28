using ReinforcementRace
ga_state = GeneticAlgorithmState(0.7, LinearCombinationCrossover())
pgs = PolicyGradientState(discount_factor=0.3)
world = World(pgs, ncars = 15)
state = GUIState(pgs, world)
create_window(state)
