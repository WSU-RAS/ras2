#!/usr/bin/env python

from lib import WaterPlantsDag, WalkDogDag, TakeMedicationDag
from lib import Items


def check_sequence(graph, seq=[], task_count=0, task_num=-1):
    if seq == []:
        return 0, False, graph['label'], False, graph['label']

    # Y intersection, pick a branch
    if 'Y' in graph:
        long_path = None
        for branch in graph['Y']:
            if seq[0] in branch:
                path = _check_sequence(branch, seq, task_count, task_num-1, label=branch['label'])
                if long_path is None:
                    long_path = path
                    continue
                if path[0] > long_path[0]:
                    long_path = path
                if path[1] and not long_path[1]:
                    long_path = path
        return long_path

    if seq[0] not in graph:
        return 0, False, graph['label'], False, graph['label']

    if len(seq) == 1:
        return 0, True, graph['label'], False, graph['next']

    return _check_sequence(graph[seq[0]], seq, task_count+1, task_num-1, label=graph['label'])

def _check_sequence(graph, seq=[], task_count=0, task_num=-1, label=None):
    # Y intersection, pick a branch
    if 'Y' in graph:
        long_path = None
        for branch in graph['Y']:
            if seq[0] in branch:
                path = _check_sequence(branch, seq, task_count, task_num, label=branch['label'])
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
        return task_count, False, graph['label'], False, graph['label']

    if task_count == task_num and seq[0] == graph['Done']:
        return task_count, True, graph['label'], True, graph['next']

    if len(seq) == 1:
        if type(graph[seq[0]]) is dict:
            return task_count, True, graph['label'], False, graph['next']
        return task_count-1, True, label, False, graph['label']

    if type(graph[seq[0]]) is dict:
        return _check_sequence(graph[seq[0]], seq, task_count+1, task_num, graph['label'])

    return _check_sequence(graph, seq[1:], task_count, task_num, label)



if __name__ == '__main__':

    print(check_sequence(
        WaterPlantsDag.taskStart,
        seq=['W', 'S', 'P1', 'P2', 'P3', 'S'],
        task_num=WaterPlantsDag.numTasks))
    print(check_sequence(
        WaterPlantsDag.taskStart,
        seq=['W', 'S', 'P1', 'P2', 'P3', 'S', 'W'],
        task_num=WaterPlantsDag.numTasks))

    print(check_sequence(
        WalkDogDag.taskStart,
        seq=['U', 'U', 'U'],
        task_num=WalkDogDag.numTasks))

    print(check_sequence(
        TakeMedicationDag.taskStart,
        seq=['F', 'F', 'C', 'C', 'F', 'F', 'C', 'F', 'S', 'S',
             'F', 'C', 'C', 'C', 'F', 'F', 'C', 'C', 'M', 'CH',
             'CH', 'C', 'F', 'F', 'F', 'M', 'M', 'C', 'C', 'C',
             'CH', 'C', 'CH', 'M', 'F', 'C', 'C', 'M', 'F', 'M',
             'F', 'M', 'C', 'F', 'F', 'S', 'C', 'C', 'F', 'M',
             'M', 'F', 'C', 'M', 'C', 'C', 'G', 'M', 'G', 'M'],
        task_num=TakeMedicationDag.numTasks))

