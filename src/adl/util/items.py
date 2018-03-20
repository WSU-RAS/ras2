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
        'EST096': 'umbrella_1',
        'EST097': 'umbrella_2',
        'EST073': 'leash_1',
        'EST074': 'leash_2',
        'EST030': 'keys_1',
        'EST123': 'keys_2',
        'EST122': 'dog_1',
        'EST100': 'dog_2',
        'EST071': 'door_1',
        'EST023': 'door_2',
        'D001': 'door__', # Smart Home sensor

        'EST098': 'food_1',
        'EST099': 'food_2',
        'EST079': 'cup_1',
        'EST080': 'cup_2',
        'EST089': 'medication_1',
        'EST090': 'medication_2',
        'EST075': 'garbage_1',
        'EST076': 'garbage_2',
        'D011': 'garbage__', # Smart Home sensor
        'EST011': 'chair_1',
        'EST078': 'chair_2',

        'EST022': 'water_can_1',
        'EST124': 'water_can_2',
        'EST024': 'side_table_plant_1',
        'EST086': 'side_table_plant_2',
        'EST087': 'windowsill_plant_1',
        'EST088': 'windowsill_plant_2',
        'EST083': 'coffee_table_plant_1',
        'EST084': 'coffee_table_plant_2',
        'EST093': 'sink_tap_1',
        'EST121': 'sink_tap_2',
    }
