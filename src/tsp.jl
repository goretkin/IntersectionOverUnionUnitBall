using PyCall: pyimport

pywrapcp = pyimport("ortools.constraint_solver.pywrapcp")
routing_enums_pb2 = pyimport("ortools.constraint_solver.routing_enums_pb2")

function setup_solve_tsp(distance_callback, n_nodes)
# Create the routing index manager.
  manager = let n_nodes = n_nodes, n_vehicles=1, start_idx=0
    pywrapcp.RoutingIndexManager(n_nodes, n_vehicles, start_idx)
  end

  # Create Routing Model.
  routing = pywrapcp.RoutingModel(manager)
  transit_callback_index = routing.RegisterTransitCallback(distance_callback)
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

  # Setting first solution heuristic.
  search_parameters = pywrapcp.DefaultRoutingSearchParameters()
  search_parameters.first_solution_strategy = (
      routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

  # Solve the problem.
  assignment = routing.SolveWithParameters(search_parameters)
  return (assignment=assignment, routing=routing, manager=manager)
end

function extract_tour(tsp_problem)
  tour_index = [tsp_problem.routing.Start(0)]

  while !tsp_problem.routing.IsEnd(tour_index[end])
    index = tour_index[end]
    index = tsp_problem.assignment.Value(tsp_problem.routing.NextVar(index))
    push!(tour_index, index)
  end
  tour_node = map(tsp_problem.manager.IndexToNode, tour_index)
  return tour_node
end




