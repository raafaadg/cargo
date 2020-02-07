import numpy as np
import pandas as pd
from math import radians, cos, sin, asin, sqrt

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def haversine(a, b):
    r = 6371
    lon1, lat1, lon2, lat2 = map(radians, [ a[1], a[0], b[1], b[0]])
    dist_lon = lon2 - lon1
    dist_lat = lat2 - lat1
    hav = sin(dist_lat/2)**2 + cos(lat1) * cos(lat2) * sin(dist_lon/2)**2
    return round((2 * r * asin( sqrt(hav) )))

def calcula_distancias(df):
    return [list([int(b) for b in a]) for a in df[['latitude','longitude']].apply(lambda x: df.apply(lambda y: haversine(x, y),axis=1) ,axis=1).values]

def gera_time_windows(df):
    return [(int(a[0]),int(a[1])) for a in df[['inicio_janela','termino_janela']].values]

def gera_volumetria(df):
    return [int(a) for a in df.volumetria.values]

def gera_peso(df):
    return [int(a) for a in df.peso.values]

def create_data_model():
    df = pd.read_csv('data.csv')
    data = {}
    data['distance_matrix'] = calcula_distancias(df)
    data['time_matrix'] = calcula_distancias(df)
    data['time_windows'] = gera_time_windows(df)
    data['demands'] = gera_volumetria(df)
    data['demands_p'] = gera_peso(df)
    data['vehicle_capacities'] = [55, 80, 100]
    data['vehicle_capacities_peso'] = [50, 70, 90]
    data['num_vehicles'] = 3
    data['vehicle_load_time'] = 0
    data['vehicle_unload_time'] = 0
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution):
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Rota para o Veiculo {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distancia da rota: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Distancia máxima de rotas: {}m'.format(max_route_distance))

def main():
    data = create_data_model()

    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  
        3000,  
        True,  
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(data, manager, routing, solution)

if __name__ == '__main__':
    main()