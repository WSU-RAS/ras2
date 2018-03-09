#!/usr/bin/env python

# Decode item numbers are based on Estimote Pairings and Script 2.0
# Experiment Date: 2018-02-13


class Items(object):

    encode = {
        'water_can': (0, 'W'),
        'sink_tap': (1, 'S'),
        'windowsill_plant': (2, 'P1'),
        'coffee_table_plant': (3, 'P2'),
        'side_table_plant': (4, 'P3'),
        'umbrella': (5, 'U'),
        'leash': (6, 'L'),
        'keys': (7, 'K'),
        'dog': (8, 'D'),
        'door': (9, 'DR'),
        'food': (10, 'F'),
        'cup': (11, 'C'),
        'medication': (12, 'M'),
        'chair': (13, 'CH'),
        'garbage': (14, 'G')
    }

    decode = {
        96: 'umbrella_1',
        97: 'umbrella_2',
        73: 'leash_1',
        74: 'leash_2',
        30: 'keys_1',
        123: 'keys_2',
        122: 'dog_1',
        100: 'dog_2',
        71: 'door_1',
        23: 'door_2',

        98: 'food_1',
        99: 'food_2',
        79: 'cup_1',
        80: 'cup_2',
        89: 'medication_1',
        90: 'medication_2',
        75: 'garbage_1',
        76: 'garbage_2',
        11: 'chair_1',
        78: 'chair_2',

        22: 'water_can_1',
        124: 'water_can_2',
        24: 'side_table_plant_1',
        86: 'side_table_plant_2',
        87: 'windowsill_plant_1',
        88: 'windowsill_plant_2',
        83: 'coffee_table_plant_1',
        84: 'coffee_table_plant_2',
        93: 'sink_tap_1',
        121: 'sink_tap_2',
    }
