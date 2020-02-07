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
    data['depot_capacity'] = 0
    
    data['vehicle_load_time'] = 0
    data['vehicle_unload_time'] = 0
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, assignment):
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Rota por Veiculo {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Tempo({1},{2}) -> '.format(
                manager.IndexToNode(index), assignment.Min(time_var),
                assignment.Max(time_var))
            index = assignment.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += '{0} Tempo({1},{2})\n'.format(manager.IndexToNode(index),
                                                    assignment.Min(time_var),
                                                    assignment.Max(time_var))
        plan_output += 'Tempo da Rota: {}min\n'.format(
            assignment.Min(time_var))
        print(plan_output)
        total_time += assignment.Min(time_var)
    print('Tempo total de todas as rotas: {}min'.format(total_time))


def main():
    data = create_data_model()

    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])

    routing = pywrapcp.RoutingModel(manager)


    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        0,  
        500,  
        False, 
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])

    
    # Adiciona restrição de Volume
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  
        data['vehicle_capacities'], 
        True, 
        'Capacity')
    
    # Adiciona restrição de Peso
    def peso_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands_p'][from_node]

    peso_callback_index = routing.RegisterUnaryTransitCallback(
        peso_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  
        data['vehicle_capacities_peso'],  
        True,  
        'Capacity')
    
    solver = routing.solver()
    intervals = []
    for i in range(data['num_vehicles']):
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),
                data['vehicle_load_time'], 'depot_interval'))
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),
                data['vehicle_unload_time'], 'depot_interval'))


    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        print_solution(data, manager, routing, assignment)
    else:
        print('No solution found !')


if __name__ == '__main__':
    main()