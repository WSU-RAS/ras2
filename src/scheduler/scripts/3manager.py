from casas.publish import PublishToCasas

class Scheduler:

    def __init__(self):
        self.startup()

    def startup(self):
        self.state = 'idle'

        # Listen to casas here

        
