import json

class Data_Control:
    def __init__(self):
        self.path = '/home/ovali/catkin_ws/src/onder_lift/src/onder_grup/'

    def read(self, file_name):
        with open(f'{self.path}{file_name}', 'r') as readfile:
            obj = json.loads(readfile.read())
        return obj

    def write(self, file_name, obj):
        with open(f'{self.path}{file_name}', 'w') as writefile:
            writefile.write(json.dumps(obj, indent=2))