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
        'retrive_water': 0,
        'fill_water': 1,
        'water_plant2': 2,
        'water_plant3': 3,
        'rinse_can': 4,
        'return_can': 5,
        'Done': 6 
    }
    subtask_info = {
        0: ('Retrieve water can', 'watercan'),
        1: ('Fill water can', 'watercan'),
        2: ('Water coffee table plant', 'plantcoffee'),
        3: ('Water side table plant', 'plantside'),
        4: ('Rinse water can', 'watercan'),
        5: ('Return water can', 'watercan'),
        6: ('Completed', None)
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
        'retrieve_umbrella': 0,
        'retrieve_leash': 1,
        'retrieve_keys': 2,
        'leash_dog': 3,
        'exit_house': 4,
        'Done': 5
    }
    subtask_info = {
        0: ('Retrieve Umbrella', 'umbrella'),
        1: ('Retrieve Leash', 'leash'),
        2: ('Retrieve Keys', 'keys'),
        3: ('Leash Dog', 'dog'),
        4: ('Exit', None),
        5: ('Completed', None)
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
        'retrieve_food': 0,
        'retrieve_cup': 1,
        'fill_cup': 2,
        'retrieve_med': 3,
        'sit_chair': 4,
        'eat_food': 5,
        'take_med': 6,
        'drink_water': 7,
        'stand_up': 8,
        'return_med': 9,
        'rinse_cup': 10,
        'throw_garbage': 11,
        'Done': 12
    }
    subtask_info = {
        0: ('Retrieve food', 'food'),
        1: ('Retrieve cup', 'glass'),
        2: ('Fill cup', 'glass'),
        3: ('Retrieve medication', 'pillbottle'),
        4: ('Sit chair', None),
        5: ('Eat food', 'food'),
        6: ('Take medication', 'pillbottle'),
        7: ('Drink water', 'glass'),
        8: ('Stand up', None),
        9: ('Return medication', 'pillbottle'),
        10: ('Rinse cup', 'glass'),
        11: ('Throw garbage', 'food'),
        12: ('Completed', None)
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
