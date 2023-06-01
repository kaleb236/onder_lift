import sys
sys.path.append("/home/ovali/catkin_ws/src/onder_lift/src/")
from data_control import Data_Control

stylesheet = '''
        QPushButton{color: white;
        }
'''
obj = Data_Control().read('styles/stylesheets.json')
obj['active_style'] = stylesheet
Data_Control().write('styles/stylesheets.json', obj)
