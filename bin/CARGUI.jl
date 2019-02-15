using ReinforcementRace
ga_state = GeneticAlgorithmState(0.7, 0.9)
nn_state = BackPropState()
world = World(ncars = 50)
state = GUIState(nn_state, world)
create_window(state)
