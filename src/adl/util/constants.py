#!/usr/bin/env python

class Goal:
    size = 3
    BASE = 0
    HUMAN = 1
    OBJECT = 2
    types = {
        BASE: "Go to base",
        HUMAN: "Find human",
        OBJECT: "Guide to object"
    }

class Task:
    size = 3
    WATER_PLANTS = 0
    TAKE_MEDS = 1
    WALK_DOG = 2
    types = {
        WATER_PLANTS: "Watering plants task",
        TAKE_MEDS: "Taking medication with food task",
        WALK_DOG: "Bring dog for a walk task"
    }
