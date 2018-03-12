#!/usr/bin/env python
'''
W   watercan                F   food/snack
S   sink                    C   cup
P1  windowsill plant        M   medication
P2  coffee table plant      CH  chair
P3  side table plant        G   garbage
U   umbrella
L   leash
K   keys
D   dog
DR  door
'''

class WaterPlantsDag(object):
    subtask = {
        'retrive_water': {
            'label': 'Retrieve Water can', 
            'index': 0},
        'fill_water': {
            'label': 'Fill Water can', 
            'index': 1},
        'water_plant2': {
            'label': 'Water Coffee table plant', 
            'index': 2},
        'water_plant3': {
            'label': 'Water Side table plant', 
            'index': 3},
        'rinse_can': {
            'label': 'Rinse Water can', 
            'index': 4},
        'return_can': {
            'label': 'Return Water can', 
            'index': 5},
        'Done': {
            'label': 'Completed', 
            'index': 6} 
    }
    num_tasks = 6
    return_can = {
        'S': None,
        'W': None,
        'Done': 'W',
        'current': 'return_can',
        'next': 'Done'
    }
    rinse_can = {
        'P3': None,
        'W': None,
        'S': return_can,
        'current': 'rinse_can',
        'next': 'return_can'
    }
    water_plant3 = {
        'P2': None,
        'W': None,
        'P3': rinse_can,
        'current': 'water_plant3',
        'next': 'rinse_can'
    }
    water_plant2 = {
        'S': None,
        'W': None,
        'P2': water_plant3,
        'current': 'water_plant2',
        'next': 'water_plant3'
    }
    fill_water = {
        'W': None,
        'S': water_plant2,
        'current': 'fill_water',
        'next': 'water_plant2'
    }
    taskStart = {
        'W': fill_water,
        'current': 'retrieve_water',
        'next': 'fill_water'
    }

class WalkDogDag(object):
    subtask = {
        'retrieve_umbrella': {
            'label': 'Retrieve Umbrella',
            'index': 0},
        'retrieve_leash': {
            'label': 'Retrieve Leash',
            'index': 1},
        'retrieve_keys': {
            'label': 'Retrieve Keys',
            'index': 2},
        'leash_dog': {
            'label': 'Leash Dog',
            'index': 3},
        'exit_house': {
            'label': 'Exit',
            'index': 4},
        'Done': {
            'label': 'Completed',
            'index': 5}
    }
    num_tasks = 5
    exit_house = {
        'U': None,
        'L': None,
        'K': None,
        'D': None,
        'DR': None,
        'Done': 'DR',
        'current': 'exit_house',
        'next': 'Done'
    }
    leash_dog = {
        'U': None,
        'L': None,
        'K': None,
        'D': exit_house,
        'current': 'leash_dog',
        'next': 'exit_house'
    }
    retrieve_keys = {
        'U': None,
        'L': None,
        'K': leash_dog,
        'current': 'retrieve_keys',
        'next': 'leash_dog'
    }
    retrieve_leash = {
        'U': None,
        'L': retrieve_keys,
        'current': 'retrieve_leash',
        'next': 'retrieve_keys'
    }
    taskStart = {
        'U': retrieve_leash,
        'current': 'retrieve_umbrella',
        'next': 'retrieve_leash'
    }

class TakeMedicationDag(object):
    subtask = {
        'retrieve_food': {
            'label': 'Retrieve Food',
            'index': 0},
        'retrieve_cup': {
            'label': 'Retrieve Cup',
            'index': 1},
        'fill_cup': {
            'label': 'Fill Cup',
            'index': 2},
        'retrieve_med': {
            'label': 'Retrieve Medication',
            'index': 3},
        'sit_chair': {
            'label': 'Sit Chair',
            'index': 4},
        'eat_food': {
            'label': 'Eat Food',
            'index': 5},
        'take_med': {
            'label': 'Take Medication',
            'index': 6},
        'drink_water': {
            'label': 'Drink Water',
            'index': 7},
        'stand_up': {
            'label': 'Stand Up',
            'index': 8},
        'return_med': {
            'label': 'Return Medication',
            'index': 9},
        'rinse_cup': {
            'label': 'Rinse Cup',
            'index': 10},
        'throw_garbage': {
            'label': 'Throw Garbage',
            'index': 11},
        'Done': {
            'label': 'Completed',
            'index': 12}
    }
    num_tasks = 12
    throw_garbage = {
        'S': None,
        'F': None,
        'C': None,
        'G': None,
        'Done': 'G',
        'current': 'throw_garbage',
        'next': 'Done'
    }
    rinse_cup = {
        'M': None,
        'F': None,
        'C': None,
        'S': throw_garbage,
        'current': 'rinse_cup',
        'next': 'throw_garbage'
    }
    return_med = {
        'CH': None,
        'F': None,
        'C': None,
        'M': rinse_cup,
        'current': 'return_med',
        'next': 'rinse_cup'
    }
    stand_up = {
        'F': None,
        'M': None,
        'C': None,
        'CH': return_med,
        'current': 'stand_up',
        'next': 'return_med'
    }
    drink_water = {
        'F': None,
        'M': None,
        'C': stand_up,
        'current': 'drink_water',
        'next': 'stand_up'
    }
    take_med = {
        'F': None,
        'C': None,
        'M': drink_water,
        'current': 'take_med',
        'next': 'drink_water'
    }
    eat_food = {
        'C': None,
        'CH': None,
        'F': take_med,
        'current': 'eat_food',
        'next': 'take_med'
    }
    sit_chair = {
        'M': None,
        'C': None,
        'F': None,
        'CH': eat_food,
        'current': 'sit_chair',
        'next': 'eat_food'
    }
    retrieve_med = {
        'F': None,
        'C': None,
        'S': None,
        'M': sit_chair,
        'current': 'retrieve_med',
        'next': 'sit_chair'
    }
    fill_cup = {
        'F': None,
        'C': None,
        'S': retrieve_med,
        'current': 'fill_cup',
        'next': 'retrieve_med'
    }
    retrieve_cup = {
        'F': None,
        'C': fill_cup,
        'current': 'retrieve_cup',
        'next': 'fill_cup'
    }
    taskStart = {
        'F': retrieve_cup,
        'current': 'retrieve_food',
        'next': 'retrieve_cup'
    }


if __name__ == '__main__':
    w = WaterPlantsDag()
    print(w.fill_water['current'])
    print(w.fill_water['S']['P2']['P3']['S']['Done'])

    wd = WalkDogDag()
    print(wd.retrieve_leash['L']['next'])
    print(wd.retrieve_leash['L']['K']['D']['Done'])
