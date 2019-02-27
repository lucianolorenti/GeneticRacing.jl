using ReinforcementRace
ga_state = GeneticAlgorithmState(0.7, LinearCombinationCrossover())
pgs = PolicyGradientState(discount_factor=0.5)
world = World(pgs, ncars = 50)
state = GUIState(pgs, world)
create_window(state)
