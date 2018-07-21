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
    subtaskName = {
        0: 'Retrieve Water can',
        1: 'Fill Water can',
        2: 'Water Windowsill plant',
        3: 'Water Coffee table plant',
        4: 'Water Side table plant',
        5: 'Rinse Water can',
        6: 'Return Water can',
        7: 'Completed'
    }
    numTasks = 7
    task6 = {
        'S': None,
        'W': None,
        'Done': 'W',
        'label': subtaskName[6],
        'next': subtaskName[7]
    }
    task5 = {
        'P3': None,
        'W': None,
        'S': task6,
        'label': subtaskName[5],
        'next': subtaskName[6]
    }
    task4 = {
        'P2': None,
        'W': None,
        'P3': task5,
        'label': subtaskName[4],
        'next': subtaskName[5]
    }
    task3 = {
        'P1': None,
        'W': None,
        'P2': task4,
        'label': subtaskName[3],
        'next': subtaskName[4]
    }
    task2 = {
        'S': None,
        'W': None,
        'P1': task3,
        'label': subtaskName[2],
        'next': subtaskName[3]
    }
    task1 = {
        'W': None,
        'S': task2,
        'label': subtaskName[1],
        'next': subtaskName[2]
    }
    taskStart = {
        'W': task1,
        'label': subtaskName[0],
        'next': subtaskName[1]
    }

class WalkDogDag(object):
    subtaskName = {
        0: 'Retrieve Umbrella',
        1: 'Retrieve Leash',
        2: 'Retrieve Keys',
        3: 'Leash Dog',
        4: 'Exit',
        5: 'Completed'
    }
    numTasks = 5
    task4 = {
        'U': None,
        'L': None,
        'K': None,
        'D': None,
        'DR': None,
        'Done': 'DR',
        'label': subtaskName[4],
        'next': subtaskName[5]
    }
    task3 = {
        'U': None,
        'L': None,
        'K': None,
        'D': task4,
        'label': subtaskName[3],
        'next': subtaskName[4]
    }
    task2 = {
        'U': None,
        'L': None,
        'K': task3,
        'label': subtaskName[2],
        'next': subtaskName[3]
    }
    task1 = {
        'U': None,
        'L': task2,
        'label': subtaskName[1],
        'next': subtaskName[2]
    }
    taskStart = {
        'U': task1,
        'label': subtaskName[0],
        'next': subtaskName[1]
    }

class TakeMedicationDag(object):
    subtaskName = {
        0: 'Retrieve Food',
        1: 'Retrieve Cup',
        2: 'Fill Cup',
        3: 'Retrieve Medication',
        4: 'Sit Chair',
        5: 'Eat Food',
        6: 'Take Medication',
        7: 'Drink Water',
        8: 'Stand Up',
        9: 'Return Medication',
        10: 'Rinse Cup',
        11: 'Throw Garbage',
        12: 'Completed'
    }
    numTasks = 12
    task11 = {
        'S': None,
        'F': None,
        'C': None,
        'G': None,
        'Done': 'G',
        'label': subtaskName[11],
        'next': subtaskName[12]
    }
    task10 = {
        'M': None,
        'F': None,
        'C': None,
        'S': task11,
        'label': subtaskName[10],
        'next': subtaskName[11]
    }
    task9 = {
        'CH': None,
        'F': None,
        'C': None,
        'M': task10,
        'label': subtaskName[9],
        'next': subtaskName[10]
    }
    task8 = {
        'F': None,
        'M': None,
        'C': None,
        'CH': task9,
        'label': subtaskName[8],
        'next': subtaskName[9]
    }
    task7 = {
        'F': None,
        'M': None,
        'C': task8,
        'label': subtaskName[7],
        'next': subtaskName[8]
    }
    task6 = {
        'F': None,
        'C': None,
        'M': task7,
        'label': subtaskName[6],
        'next': subtaskName[7]
    }
    task5 = {
        'C': None,
        'CH': None,
        'F': task6,
        'label': subtaskName[5],
        'next': subtaskName[6]
    }
    task4 = {
        'M': None,
        'C': None,
        'F': None,
        'CH': task5,
        'label': subtaskName[4],
        'next': subtaskName[5]
    }
    task3 = {
        'F': None,
        'C': None,
        'S': None,
        'M': task4,
        'label': subtaskName[3],
        'next': subtaskName[4]
    }
    task2 = {
        'F': None,
        'C': None,
        'S': task3,
        'label': subtaskName[2],
        'next': subtaskName[3]
    }
    ''' cup->sink->food '''
    task2_1 = {
        'C': None,
        'S': None,
        'F': task3,
        'label': subtaskName[0],
        'next': subtaskName[3]
    }
    ''' cup->sink '''
    task1_2_2 = {
        'C': None,
        'S': task2_1,
        'label': subtaskName[2],
        'next': subtaskName[0]
    }
    ''' cup->food '''
    task1_2_1 = {
        'C': None,
        'F': task2,
        'label': subtaskName[0],
        'next': subtaskName[2]
    }
    ''' cup->Y '''
    task1_2 = {
        'Y': [task1_2_1, task1_2_2]
    }
    ''' food->cup '''
    task1_1 = {
        'F': None,
        'C': task2,
        'label': subtaskName[1],
        'next': subtaskName[2]
    }
    ''' starts with retrieving cup '''
    taskStart_2 = {
        'C': task1_2,
        'label': subtaskName[1],
        'next': subtaskName[0]
    }
    ''' starts with retrieving food '''
    taskStart_1 = {
        'F': task1_1,
        'label': subtaskName[0],
        'next': subtaskName[1]
    }
    ''' start->Y '''
    taskStart = {
        'Y': [taskStart_1, taskStart_2]
    }


if __name__ == '__main__':
    w = WaterPlantsDag()
    print(w.task1['W'])
    print(w.task1['S']['P1']['P2']['P3']['S']['Done'])

    wd = WalkDogDag()
    print(wd.task1['U'])
    print(wd.task1['L']['K']['D']['DR'])
