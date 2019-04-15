import rospy

from object_detection_msgs.srv import ObjectQuery

class Decode():

    def __init__(self):
        return None

    # Lookup table for various points n stuff
    def point(self, activity, step, state):
        # Note: Steps dont matter at the moment
        pnt = None
        
        # Activity = 0 --> Take Medicine
        if activity == '0':
            if state == '1': 
            if state == '4':
                pnt = get_object_location('base')

        # Activity = 1 --> Make Coffee
        if activity == '1':
            if state == '1': 
            if state == '4':
                pnt = get_object_location('base')

        # Activity = 2 --> Make Breakfest
        if activity == '2':
            if state == '1': 
            if state == '4':
                pnt = get_object_location('base')

        return pnt


    def get_human(self)
        loc = self.get_object_location('human')
        return loc

    def get_object_location(self, name):
        rospy.wait_for_service("query_objects")
        try:
            query = rospy.ServiceProxy("query_objects", ObjectQuery)
            result = query(name)
            return result.locations
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        return None

    def get_data(activity, error_step):
        """
        From the task number and error step, get the corresponding object name
        and video urls
        """
        object_to_find = ''  # No object for this error / step
        video_step_url = ''
        video_full_url = ''
        face_url       = 'happy-cartoon-face-hi.png'

        # Activity = 0 --> Take Medicine
        if activity == '0':
            object_to_find = ''
            video_step_url = ''
            video_full_url = ''

        # Activity = 1 --> Make Coffee
        elif activity == '1':
            object_to_find = ''
            video_step_url = ''
            video_full_url = ''

        # Activity = 2 --> Make Breakfest
        elif activity == '2':
            object_to_find = ''
            video_step_url = ''
            video_full_url = ''

        return object_to_find, video_step_url, video_full_url, face_url

