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
    side_plant = {
        'P2': None,
        'W': None,
        'P3': rinse_can,
        'current': 'side_plant',
        'next': 'rinse_can'
    }
    coffee_plant = {
        'S': None,
        'W': None,
        'P2': side_plant,
        'current': 'coffee_plant',
        'next': 'side_plant'
    }
    fill_can = {
        'W': None,
        'S': coffee_plant,
        'current': 'fill_can',
        'next': 'coffee_plant'
    }
    get_can = {
        'W': fill_can,
        'current': 'get_can',
        'next': 'fill_can'
    }
    task_start = get_can

    subtask = {
        'get_can': 0,
        'fill_can': 1,
        'coffee_plant': 2,
        'side_plant': 3,
        'rinse_can': 4,
        'return_can': 5,
        'Done': 6
    }
    subtask_info = {
        # (description, query object, current dag, next dag if error)
        0: ('Retrieve water can', 'watercan', get_can, fill_can),
        1: ('Fill water can', 'watercan', fill_can, coffee_plant),
        2: ('Water coffee table plant', 'plantcoffee', coffee_plant, rinse_can),
        3: ('Water side table plant', 'plantside', side_plant, rinse_can),
        4: ('Rinse water can', 'watercan', rinse_can, return_can),
        5: ('Return water can', 'watercan', return_can, None),
        6: ('Completed', None, None, None)
    }
    num_tasks = 6


class WalkDogDag(object):

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
    get_keys = {
        'U': None,
        'L': None,
        'K': leash_dog,
        'current': 'get_keys',
        'next': 'leash_dog'
    }
    get_leash = {
        'U': None,
        'L': get_keys,
        'current': 'get_leash',
        'next': 'get_keys'
    }
    get_umbrella = {
        'U': get_leash,
        'current': 'get_umbrella',
        'next': 'get_leash'
    }
    task_start = get_umbrella

    subtask = {
        'get_umbrella': 0,
        'get_leash': 1,
        'get_keys': 2,
        'leash_dog': 3,
        'exit_house': 4,
        'Done': 5
    }
    subtask_info = {
        # (description, query object, current dag, next dag if error)
        0: ('Retrieve Umbrella', 'umbrella', get_umbrella, get_keys),
        1: ('Retrieve Leash', 'leash', get_leash, leash_dog),
        2: ('Retrieve Keys', 'keys', get_keys, exit_house),
        3: ('Leash Dog', 'dog', leash_dog, exit_house),
        4: ('Exit', None, exit_house, None),
        5: ('Completed', None, None, None)
    }
    num_tasks = 5

class TakeMedicationDag(object):

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
    get_med = {
        'F': None,
        'C': None,
        'S': None,
        'M': sit_chair,
        'current': 'get_med',
        'next': 'sit_chair'
    }
    fill_cup = {
        'F': None,
        'C': None,
        'S': get_med,
        'current': 'fill_cup',
        'next': 'get_med'
    }
    get_cup = {
        'F': None,
        'C': fill_cup,
        'current': 'get_cup',
        'next': 'fill_cup'
    }
    get_food = {
        'F': get_cup,
        'current': 'get_food',
        'next': 'get_cup'
    }
    task_start = get_food

    subtask = {
        'get_food': 0,
        'get_cup': 1,
        'fill_cup': 2,
        'get_med': 3,
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
        # (description, query object, current dag, next dag if error)
        0: ('Retrieve food', 'food', get_food, fill_cup),
        1: ('Retrieve cup', 'glass', get_cup, fill_cup),
        2: ('Fill cup', 'glass', fill_cup, sit_chair),
        3: ('Retrieve medication', 'pillbottle', get_med),
        4: ('Sit chair', None, sit_chair, take_med),
        5: ('Eat food', 'food', eat_food, drink_water),
        6: ('Take medication', 'pillbottle', take_med, drink_water),
        7: ('Drink water', 'glass', drink_water, return_med),
        8: ('Stand up', None, stand_up, stand_up),
        9: ('Return medication', 'pillbottle', return_med, throw_garbage),
        10: ('Rinse cup', 'glass', rinse_cup, None),
        11: ('Throw garbage', 'food', throw_garbage, None),
        12: ('Completed', None, None, None)
    }
    num_tasks = 12


if __name__ == '__main__':
    w = WaterPlantsDag()
    print(w.fill_can['current'])
    print(w.fill_can['S']['P2']['P3']['S']['Done'])

    wd = WalkDogDag()
    print(wd.get_leash['L']['next'])
    print(wd.get_leash['L']['K']['D']['Done'])
