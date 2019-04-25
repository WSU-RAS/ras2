import rospy

from object_detection_msgs.srv import ObjectQuery

class Pnt():
    
    def __init__(self, x, y, w, z):
        self.x = x
        self.y = y
        self.w = w
        self.z = z

class Decode():

    def __init__(self):
        return None

    # Lookup table for various points n stuff
    def point(self, activity, step, state):
        # Note: Steps dont matter at the moment
        pnt = None

        # Activity = 0 --> Take Medicine
        if activity == 'Take_Medicine':
            if state == '1':
                pnt = self.get_object_location('meds1')
            elif state == '2':
                pnt = self.get_object_location('meds2')
            elif state == '3': 
                pnt = self.get_object_location('coffeeobj')

        # Activity = 1 --> Make Coffee
        if activity == 'Make_Coffee':
            if state == '1': 
                pnt = self.get_object_location('coffee1')
            elif state == '2': 
                pnt = self.get_object_location('coffee2')


        # Activity = 2 --> Make Breakfest
        if activity == 'Eat':
            if state == '1':
                pnt = self.get_object_location('food1')
            elif state == '2':
                pnt = self.get_object_location('food2')

        if state == '4':
            pnt = self.get_object_location('base')

        return pnt


    def get_human(self):
        loc = self.get_object_location('human')
        return loc

    def get_object_location(self, name):
        # Lookup Table for points (used to be a service)
        data = None

        if name == 'meds1':
            data = Pnt(0,0,0,1)
        elif name == 'meds2':
            data = Pnt(0,0,0,1)
        elif name == 'coffeeobj':
            data = Pnt(0,0,0,1)

        elif name == 'coffee1':
            data = Pnt(0,0,0,1)
        elif name == 'coffee2':
            data = Pnt(0,0,0,1)

        elif name == 'food1':
            data = Pnt(0,0,0,1)
        elif name == 'food2':
            data = Pnt(0,0,0,1)

        elif name == 'base':
            data = Pnt(0,0,0,1)

        return data

    def get_data(self, activity, error_step):
        """
        From the task number and error step, get the corresponding object name
        and video urls
        """
        object_to_find = ''  # No object for this error / step
        video_step_url = ''
        video_full_url = ''
        face_url       = 'happy-cartoon-face-hi.png'

        # Activity = 0 --> Take Medicine
        if activity == 'Take_Medicine':
            object_to_find = 'Sink'
            video_step_url = 'error/takemedicine.error45.mp4'
            video_full_url = 'full/takemedicine.full.mp4'

        # Activity = 1 --> Make Coffee
        elif activity == 'Make_Coffee':
            object_to_find = ''
            video_step_url = 'error/makecoffee.error5.mp4 '
            video_full_url = 'full/makecoffee.full.mp4 '

        # Activity = 2 --> Make Breakfest
        elif activity == 'Eat':
            object_to_find = ''
            video_step_url = 'error/eat.error4.mp4 '
            video_full_url = 'full/eat.full.mp4'

        return object_to_find, video_step_url, video_full_url, face_url

