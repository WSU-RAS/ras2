#!/usr/bin/env python
'''
Estimotes:
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
PL  pills

Location Change:
L>K living_room to kitchen
K>L kitchen to living_room
L>H living_room to hallway
H>KH hallway to kitchen_hallway
KH>H kitchen_hallway to kitchen
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
    water_plantside = {
        'P2': None,
        'W': None,
        'P3': rinse_can,
        'current': 'water_plantside',
        'next': 'rinse_can'
    }
    water_plantcoffee = {
        'S': None,
        'W': None,
        'P2': water_plantside,
        'current': 'water_plantcoffee',
        'next': 'water_plantside'
    }
    fill_can = {
        'W': None,
        'S': water_plantcoffee,
        'current': 'fill_can',
        'next': 'water_plantcoffee'
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
        'water_plantcoffee': 2,
        'water_plantside': 3,
        'rinse_can': 4,
        'return_can': 5,
        'Done': 6
    }
    subtask_info = {
        # Note: for now we'll just show the button "guide me to object" for objects to
        # be retreived since we aren't tracking them as they're moving.
        #   (description              , query object , code, current dag , next dag if error)
        0: ('Retrieve water can'      , 'watercan'   , 'W' , get_can          , fill_can),
        1: ('Fill water can'          , 'sink'       , 'S' , fill_can         , water_plantcoffee),
        2: ('Water coffee table plant', 'plantcoffee', 'P2', water_plantcoffee, rinse_can),
        3: ('Water side table plant'  , 'plantside'  , 'P3', water_plantside  , rinse_can),
        4: ('Rinse water can'         , 'sink'       , 'S' , rinse_can        , return_can),
        5: ('Return water can'        , None         , 'W' , return_can       , None),
        6: ('Completed'               , None         , None, None             , None)
    }
    num_tasks = 6

class WaterPlantsWithLocationDag(object):

    return_can = {
        'S': None,
        'W': None,
        'K>L': None,
        'Done': 'K>L',
        'current': 'return_can',
        'next': 'Done'
    }
    rinse_can = {
        'P3': None,
        'W': None,
        'L>K': None,
        'S': return_can,
        'current': 'rinse_can',
        'next': 'return_can'
    }
    water_plantside = {
        'P2': None,
        'W': None,
        'P3': rinse_can,
        'current': 'water_plantside',
        'next': 'rinse_can'
    }
    water_plantcoffee = {
        'S': None,
        'W': None,
        'K>L': None,
        'P2': water_plantside,
        'current': 'water_plantcoffee',
        'next': 'water_plantside'
    }
    fill_can = {
        'W': None,
        'L>K': None,
        'S': water_plantcoffee,
        'current': 'fill_can',
        'next': 'water_plantcoffee'
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
        'water_plantcoffee': 2,
        'water_plantside': 3,
        'rinse_can': 4,
        'return_can': 5,
        'Done': 6
    }
    subtask_info = {
        # Note: for now we'll just show the button "guide me to object" for objects to
        # be retreived since we aren't tracking them as they're moving.
        #   (description              , query object , code , current dag , next dag if error)
        0: ('Retrieve water can'      , 'watercan'   , 'W'  , get_can          , fill_can),
        1: ('Fill water can'          , 'sink'       , 'S'  , fill_can         , water_plantcoffee),
        2: ('Water coffee table plant', 'plantcoffee', 'P2' , water_plantcoffee, rinse_can),
        3: ('Water side table plant'  , 'plantside'  , 'P3' , water_plantside  , rinse_can),
        4: ('Rinse water can'         , 'sink'       , 'S'  , rinse_can        , return_can),
        5: ('Return water can'        , None         , 'K>L', return_can       , None),
        6: ('Completed'               , None         , None , None             , None)
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
        #   (description       , query object, code, current dag , next dag if error)
        0: ('Retrieve Umbrella', 'umbrella'  , 'U' , get_umbrella, get_keys),
        1: ('Retrieve Leash'   , 'leash'     , 'L' , get_leash   , leash_dog),
        2: ('Retrieve Keys'    , 'keys'      , 'K' , get_keys    , exit_house),
        3: ('Leash Dog'        , 'dog'       , 'D' , leash_dog   , exit_house),
        4: ('Exit'             , None        , 'DR', exit_house  , None),
        5: ('Completed'        , None        , None, None        , None)
    }
    num_tasks = 5

class WalkDogWithLocationDag(object):

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
        'H>KH': None,
        'KH>H': None,
        'D': exit_house,
        'current': 'leash_dog',
        'next': 'exit_house'
    }
    get_keys = {
        'U': None,
        'L': None,
        'KH>H': None,
        'H>KH': None,
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
        'L>H': None,
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
        #   (description       , query object, code, current dag , next dag if error)
        0: ('Retrieve Umbrella', 'umbrella'  , 'U' , get_umbrella, get_keys),
        1: ('Retrieve Leash'   , 'leash'     , 'L' , get_leash   , leash_dog),
        2: ('Retrieve Keys'    , 'keys'      , 'K' , get_keys    , exit_house),
        3: ('Leash Dog'        , 'dog'       , 'D' , leash_dog   , exit_house),
        4: ('Exit'             , None        , 'DR', exit_house  , None),
        5: ('Completed'        , None        , None, None        , None)
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
        'CH': None,
        'M': None,
        'F': None,
        'C': None,
        'S': throw_garbage,
        'current': 'rinse_cup',
        'next': 'throw_garbage'
    }
    stand_up = {
        'F': None,
        'M': None,
        'C': None,
        'CH': rinse_cup,
        'current': 'stand_up',
        'next': 'rinse_cup'
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
        #'C': None,
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
        'rinse_cup': 9,
        'throw_garbage': 10,
        'Done': 11
    }
    subtask_info = {
        #   (description          , query object, code, current dag  , next dag if error)
        0:  ('Retrieve food'      , 'food'      , 'F' , get_food     , fill_cup),
        1:  ('Retrieve cup'       , 'glass'     , 'C' , get_cup      , fill_cup),
        2:  ('Fill cup'           , 'sink'      , 'S' , fill_cup     , sit_chair),
        3:  ('Retrieve medication', 'pillbottle', 'M' , get_med      , eat_food),
        4:  ('Sit chair'          , None        , 'CH', sit_chair    , take_med),
        5:  ('Eat food'           , None        , 'F' , eat_food     , drink_water),
        6:  ('Take medication'    , 'pills'     , 'M' , take_med     , stand_up),
        7:  ('Drink water'        , None        , 'C' , drink_water  , rinse_cup),
        8:  ('Stand up'           , None        , 'CH', stand_up     , throw_garbage),
        9:  ('Rinse cup'          , 'sink'      , 'C' , rinse_cup    , None),
        10: ('Throw garbage'      , None        , 'G' , throw_garbage, None),
        11: ('Completed'          , None        , None, None         , None)
    }
    num_tasks = 11

class TakeMedicationWithLocationDag(object):

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
        'CH': None,
        'M': None,
        'F': None,
        'C': None,
        'L>K': None,
        'S': throw_garbage,
        'current': 'rinse_cup',
        'next': 'throw_garbage'
    }
    stand_up = {
        'F': None,
        'M': None,
        'C': None,
        'CH': rinse_cup,
        'current': 'stand_up',
        'next': 'rinse_cup'
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
        #'C': None,
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
    put_med = {
        'M': None,
        'K>L': sit_chair,
        'current': 'put_med',
        'next': 'sit_chair'
    }
    get_med = {
        'F': None,
        'C': None,
        'L>K': None,
        'M': put_med,
        'current': 'get_med',
        'next': 'put_med'
    }
    put_food_cup = {
        'F': None,
        'C': None,
        'S': None,
        'K>L': get_med,
        'current': 'put_food_cup',
        'next': 'get_med'
    }
    fill_cup = {
        'F': None,
        'C': None,
        'S': put_food_cup,
        'current': 'fill_cup',
        'next': 'put_food_cup'
    }
    get_cup = {
        'F': None,
        'C': fill_cup,
        'current': 'get_cup',
        'next': 'fill_cup'
    }
    get_food = {
        'L>K': None,
        'F': get_cup,
        'current': 'get_food',
        'next': 'get_cup'
    }
    task_start = get_food

    subtask = {
        'get_food': 0,
        'get_cup': 1,
        'fill_cup': 2,
        'put_food_cup': 3,
        'get_med': 4,
        'put_med': 5,
        'sit_chair': 6,
        'eat_food': 7,
        'take_med': 8,
        'drink_water': 9,
        'stand_up': 10,
        'rinse_cup': 11,
        'throw_garbage': 12,
        'Done': 13
    }
    subtask_info = {
        #   (description            , query object, code , current dag  , next dag if error)
        0:  ('Retrieve food'        , 'food'      , 'F'  , get_food     , fill_cup),
        1:  ('Retrieve cup'         , 'glass'     , 'C'  , get_cup      , fill_cup),
        2:  ('Fill cup'             , 'sink'      , 'S'  , fill_cup     , put_food_cup),
        3:  ('Put food/cup on table', None        , 'K>L', put_food_cup , put_med),
        4:  ('Retrieve medication'  , 'pillbottle', 'M'  , get_med      , put_med),
        5:  ('Put med on table'     , None        , 'K>L', put_med      , sit_chair),
        6:  ('Sit chair'            , None        , 'CH' , sit_chair    , take_med),
        7:  ('Eat food'             , None        , 'F'  , eat_food     , drink_water),
        8:  ('Take medication'      , 'pills'     , 'M'  , take_med     , stand_up),
        9:  ('Drink water'          , None        , 'C'  , drink_water  , rinse_cup),
        10: ('Stand up'             , None        , 'CH' , stand_up     , throw_garbage),
        11: ('Rinse cup'            , 'sink'      , 'C'  , rinse_cup    , None),
        12: ('Throw garbage'        , None        , 'G'  , throw_garbage, None),
        13: ('Completed'            , None        , None , None         , None)
    }
    num_tasks = 13


if __name__ == '__main__':
    w = WaterPlantsDag()
    print(w.fill_can['current'])
    print(w.fill_can['S']['P2']['P3']['S']['Done'])

    wd = WalkDogDag()
    print(wd.get_leash['L']['next'])
    print(wd.get_leash['L']['K']['D']['Done'])
