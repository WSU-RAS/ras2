#!/usr/bin/env python

from constants import Task
from dag import WaterPlantsDag, TakeMedicationDag, WalkDogDag

# Decode item numbers are based on Estimote Pairings and Script 2.0
# Experiment Date: 2018-02-13

class TaskToDag(object):

    mapping = {
        Task.WATER_PLANTS: WaterPlantsDag,
        Task.TAKE_MEDS: TakeMedicationDag,
        Task.WALK_DOG: WalkDogDag
    }

class Items(object):

    encode = {
        'water_can': (0, 'W', [Task.WATER_PLANTS]),
        'sink_tap': (1, 'S', [Task.WATER_PLANTS, Task.TAKE_MEDS]),
        'windowsill_plant': (2, 'P1', [Task.WATER_PLANTS]),
        'coffee_table_plant': (3, 'P2', [Task.WATER_PLANTS]),
        'side_table_plant': (4, 'P3', [Task.WATER_PLANTS]),
        'umbrella': (5, 'U', [Task.WALK_DOG]),
        'leash': (6, 'L', [Task.WALK_DOG]),
        'keys': (7, 'K', [Task.WALK_DOG]),
        'dog': (8, 'D', [Task.WALK_DOG]),
        'door': (9, 'DR', [Task.WALK_DOG]),
        'food': (10, 'F', [Task.TAKE_MEDS]),
        'cup': (11, 'C', [Task.TAKE_MEDS]),
        'medication': (12, 'M', [Task.TAKE_MEDS]),
        'chair': (13, 'CH', [Task.TAKE_MEDS]),
        'garbage': (14, 'G', [Task.TAKE_MEDS])
    }

    decode = {
        'EST323': 'umbrella',
        'EST324': 'umbrella',
        'EST321': 'leash',
        'EST322': 'leash',
        'EST319': 'keys',
        'EST320': 'keys',
        'EST327': 'dog',
        'EST328': 'dog',
        'EST325': 'door',
        'EST326': 'door',
        'D001': 'door', # Smart Home sensor

        'EST317': 'food',
        'EST318': 'food',
        'EST311': 'cup',
        'EST312': 'cup',
        'EST313': 'medication',
        'EST314': 'medication',
        'EST329': 'garbage',
        'EST330': 'garbage',
        'D011': 'garbage', # Smart Home sensor
        'EST301': 'chair',
        'EST310': 'chair',

        'EST308': 'water_can',
        'EST309': 'water_can',
        'EST302': 'side_table_plant',
        'EST303': 'side_table_plant',
        'EST306': 'windowsill_plant',
        'EST307': 'windowsill_plant',
        'EST304': 'coffee_table_plant',
        'EST305': 'coffee_table_plant',
        'EST315': 'sink_tap',
        'EST316': 'sink_tap',
    }
