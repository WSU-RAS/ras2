#! /usr/bin/env python

import rospy

from actionlib import SimpleActionServer
from scheduler.msg import GotoAction, GotoFeedback, GotoResult

class Task:
    HOME = 0
    HUMAN = 1
    OBJECT = 2
    WATER_PLANTS = 3
    TAKE_MEDS = 4
    WALK_DOG = 5
    tasks = {
        HOME: "Go to home",
        HUMAN: "Find human",
        OBJECT: "Guide to object",
        WATER_PLANTS: "Watering plants task",
        TAKE_MEDS: "Taking medication with food task",
        WALK_DOG: "Bring dog for a walk task"
    }

class GotoServer:

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.goto_server = SimpleActionServer(
            'goto', GotoAction,
            execute_cb=self.goto_execute,
            auto_start=False)
        self.goto_server.start()

    def goto_execute(self, goal):
        x = 20
        y = -20
        z = 0
        
        rospy.loginfo("Executing {}".format(Task.tasks[goal.task_number]))
        rospy.loginfo("error_step={}  error_object={}".format(goal.error_step, goal.error_object))

        goto_feedback = GotoFeedback()
        while True:
            goto_feedback.x = x
            goto_feedback.y = y
            goto_feedback.z = z
            goto_feedback.status = 1
            goto_feedback.text = 'ROBOT MOVING'
            self.goto_server.publish_feedback(goto_feedback)
            self.rate.sleep()
            x -= 1
            y += 1
            if x == 0 and y == 0 and z == 0:
                break

        goto_feedback.x = x
        goto_feedback.y = y
        goto_feedback.z = z
        goto_feedback.status = 2
        goto_feedback.text = 'TASK COMPLETED'

        goto_result = GotoResult()
        goto_result.status = 2
        goto_result.is_complete = True
        self.goto_server.set_succeeded(goto_result)

if __name__ == '__main__':
    rospy.init_node('goto_server')
    server = GotoServer()
    rospy.spin()

        
