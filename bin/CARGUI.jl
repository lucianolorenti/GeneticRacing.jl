using ReinforcementRace

ga_state = GeneticAlgorithmState(0.7, 0.9)
world = World(ncars = 50)
state = GUIState(ga_state, world)
create_window(state)
