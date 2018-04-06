#!/usr/bin/env python

from util import WaterPlantsDag, WalkDogDag, TakeMedicationDag
from util import Items


def check_sequence(graph, seq=[], task_count=0, task_num=-1):
    if seq == []:
        return 0, False, graph['current'], False, graph['current']

    if task_num > 0:
        return _check_sequence(graph, seq, task_count, task_num-1, current=graph['current'])

    # Y intersection, pick a branch
    if 'Y' in graph:
        long_path = None
        for branch in graph['Y']:
            if seq[0] in branch:
                path = _check_sequence(branch, seq, task_count, task_num-1, current=branch['current'])
                if long_path is None:
                    long_path = path
                    continue
                if path[0] > long_path[0]:
                    long_path = path
                if path[1] and not long_path[1]:
                    long_path = path
        return long_path

    if seq[0] not in graph:
        return 0, False, graph['current'], False, graph['current']

    if len(seq) == 1:
        return 0, True, graph['current'], False, graph['next']

    return _check_sequence(graph[seq[0]], seq, task_count+1, task_num-1, current=graph['current'])

def _check_sequence(graph, seq=[], task_count=0, task_num=-1, current=None):
    # Y intersection, pick a branch
    if 'Y' in graph:
        long_path = None
        for branch in graph['Y']:
            if seq[0] in branch:
                path = _check_sequence(branch, seq, task_count, task_num, current=branch['current'])
                if path[1] and path[3]: # Completed
                    return path
                if long_path is None:
                    long_path = path
                    continue
                if path[0] > long_path[0]:
                    long_path = path
                if path[1] and not long_path[1]:
                    long_path = path
        return long_path

    if seq[0] not in graph:
        return task_count, False, graph['current'], False, graph['current']

    if task_count == task_num and seq[0] == graph['Done']:
        return task_count, True, graph['current'], True, graph['next']

    if len(seq) == 1:
        if type(graph[seq[0]]) is dict:
            return task_count, True, graph['current'], False, graph['next']
        return task_count-1, True, current, False, graph['current']

    if type(graph[seq[0]]) is dict:
        return _check_sequence(graph[seq[0]], seq, task_count+1, task_num, graph['current'])

    return _check_sequence(graph, seq[1:], task_count, task_num, current)



if __name__ == '__main__':

    print(check_sequence(
        WaterPlantsDag.water_plant2,
        seq=['W','W','W'],
        task_count=2,
        task_num=WaterPlantsDag.num_tasks))

    print(check_sequence(
        WaterPlantsDag.taskStart,
        seq=['W', 'S', 'P2', 'P3', 'S'],
        task_num=WaterPlantsDag.num_tasks))
    print(check_sequence(
        WaterPlantsDag.taskStart,
        seq=['W', 'S', 'P2', 'P3', 'S', 'W'],
        task_num=WaterPlantsDag.num_tasks))

    print(check_sequence(
        WalkDogDag.taskStart,
        seq=['U', 'U', 'U'],
        task_num=WalkDogDag.num_tasks))

    print(check_sequence(
        WalkDogDag.taskStart,
        seq=['U', 'L', 'K', 'D', 'DR'],
        task_num=WalkDogDag.num_tasks))

    print(check_sequence(
        TakeMedicationDag.taskStart,
        seq=['F', 'F', 'C', 'C', 'F', 'F', 'C', 'F', 'S', 'S',
             'F', 'C', 'C', 'C', 'F', 'F', 'C', 'C', 'M', 'CH',
             'CH', 'C', 'F', 'F', 'F', 'M', 'M', 'C', 'C', 'C',
             'CH', 'C', 'CH', 'M', 'F', 'C', 'C', 'M', 'F', 'M',
             'F', 'M', 'C', 'F', 'F', 'S', 'C', 'C', 'F', 'M',
             'M', 'F', 'C', 'M', 'C', 'C', 'G', 'M', 'G', 'M'],
        task_num=TakeMedicationDag.num_tasks))
