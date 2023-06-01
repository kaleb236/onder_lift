#!/usr/bin/env python3
import rospy
import sys, os
import json
import subprocess
import rosnode
sys.path.append("/home/ovali/catkin_ws/src/onder_lift/src/")
from PyQt5.QtWidgets import QApplication
from onder_ui import Ui_MainWindow

from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, Qt, pyqtSlot
from ui_functions import *

from rviz import bindings as rviz
from motion_planner.velocity_control import Twist_pub
from motion_planner.navigation import Navigation
from data_control import Data_Control

class ui_windows(QMainWindow):
    def __init__(self):
        super(ui_windows, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.path = '/home/ovali/catkin_ws/src/onder_lift'

        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: UIFunctions.print_mouse_position(self))

        self.vel_pub = Twist_pub()
        self.data_control = Data_Control()
        self.navigation = Navigation()

        self.button_clicks()
        self.initial_variables()
        UIFunctions.add_rviz(self)
    
    def initial_variables(self):
        self.ui.settings_pages.setCurrentWidget(self.ui.admin_page)
        self.ui.stackedWidget.setCurrentWidget(self.ui.home_page)
        self.current_station = QPushButton()
        self.add_maps()
        self.stations = []
        self.motion_timeout = 2
        self.motion_index = 0
        self.mapping = False
        self.current_map = ''
        self.lin_vel = 0
        self.ang_vel = 0
        self.linear_vel = 0.2
        self.angular_vel = 0.5
        UIFunctions.set_velocity(self, 0.0)
        self.X1, self.Y1 = self.ui.joy_inside.geometry().x(), self.ui.joy_inside.geometry().y()
    
    def add_maps(self):
        map_items = []
        for map_name in os.listdir(self.path+'/maps'):
            if map_name.endswith(".yaml"):
                map_items.append(os.path.splitext(map_name)[0])
        
        self.ui.maps_line_3.addItems(sorted(map_items))
    
    def button_clicks(self):
        for i in range(0, self.ui.home_gridlayout.count()):
            self.ui.home_gridlayout.itemAt(i).widget().clicked.connect(self.station_click)
        self.ui.backspace_btn.clicked.connect(self.delete_path)
        self.ui.settings_btn.clicked.connect(lambda: self.main_page_navigation(self.ui.rviz_page))
        self.ui.close_setting.clicked.connect(lambda: self.main_page_navigation(self.ui.home_page))
        self.ui.close_map.clicked.connect(lambda: self.admin_page_navigation(self.ui.admin_page))
        self.ui.new_map_3.clicked.connect(lambda: self.admin_page_navigation(self.ui.map_page))
        self.ui.start_map_2.clicked.connect(self.start_mapping)
        self.ui.save_map_2.clicked.connect(self.save_map)
        self.ui.set_initial_2.clicked.connect(self.save_initial)
        self.ui.save_btn_3.clicked.connect(self.save_stations)
        self.ui.station_9.clicked.connect(self.select_station)
        self.ui.station_10.clicked.connect(self.select_station)
        self.ui.station_11.clicked.connect(self.select_station)
        self.ui.station_12.clicked.connect(self.select_station)
        self.ui.load_map_3.clicked.connect(self.load_map)
        self.ui.set_timer.clicked.connect(self.set_timer)
        self.ui.start_planner.clicked.connect(self.start_motion)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.timer.start(5)
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.ui.joy_inside.move(self.X1, self.Y1)
            self.ang_vel, self.lin_vel = 0.00, 0.00
            UIFunctions.set_velocity(self, abs(round(self.lin_vel, 2)))
            self.vel_pub.send_vel(self.lin_vel, self.ang_vel)
            self.timer.stop()
    
    
    def get_style(self):
        with open(f'{self.path}/src/onder_grup/styles/stylesheets.json', 'r') as readfile:
            obj = json.loads(readfile.read())
        return obj

    def station_click(self):
        self.stations.append(self.sender().objectName().split('_')[1])
        self.ui.planner_label.setText('→'.join(self.stations))
    
    def delete_path(self):
        if self.stations:
            self.stations.pop()
        self.ui.planner_label.setText('→'.join(self.stations))
    
    def main_page_navigation(self, page):
        self.ui.stackedWidget.setCurrentWidget(page)
        self.ui.save_frame_2.setMaximumHeight(0)
    
    def admin_page_navigation(self, page):
        self.ui.save_frame_2.setMaximumHeight(0)
        self.ui.mapping_info_2.setText('Click on record button to start mapping')
        self.ui.settings_pages.setCurrentWidget(page)
    
    def map_server(self):
        for i in rosnode.get_node_names():
            if '/map' in i.split('_'):
                return i
    
    def start_mapping(self):
        if not self.mapping:
            self.ui.mapping_info_2.setText('Starting mapping in few seconds. Please wait...')
            self.sender().setStyleSheet(self.get_style()['stop_style'])
            UIFunctions.animations(self, 0, self.ui.save_frame_2)
            os.system(f"rosnode kill /amcl {self.map_server()}")
            self.mapping_process = subprocess.Popen(
                "roslaunch onder_lift mapping.launch", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid
            )
            QTimer.singleShot(10000, lambda: self.ui.mapping_info_2.setText('Mapping started move around the robot.'))
            self.mapping = True
        else:
            self.ui.mapping_info_2.setText('Type map name then click save')
            self.sender().setStyleSheet(self.get_style()['record_style'])
            UIFunctions.animations(self, 150, self.ui.save_frame_2)
            self.mapping = False
    
    def save_map(self):
        if self.ui.lineEdit_2.text():
            self.current_map = self.ui.lineEdit_2.text()
            self.ui.mapping_info_2.setText('Saving map. Please wait...')
            saving_map = subprocess.Popen(
                f"rosrun map_server map_saver -f {self.path}/maps/{self.ui.lineEdit_2.text()}", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid
            )
            QTimer.singleShot(5000, lambda: self.ui.mapping_info_2.setText('Map succesfully saved.'))
            QTimer.singleShot(7000, self.exit_gmapping)
        else:
            self.ui.mapping_info_2.setText('Please type map name')
    
    def exit_gmapping(self):
        self.add_maps()
        if self.current_map in [self.ui.maps_line_3.itemText(i) for i in range(self.ui.maps_line_3.count())]:
            os.system('rosnode kill /slam_gmapping')
            subprocess.Popen(
                    f"rosrun map_server map_server {self.path}/maps/{self.current_map}.yaml", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid
                )
            subprocess.Popen(
                    "roslaunch rur_navigation amcl.launch", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid
                )
            self.ui.mapping_info_2.setText("Estimate robot's starting point then click on set initial")
        else:
            self.ui.mapping_info_2.setText('Map was not successfully save please try again')
    
    def save_initial(self):
        obj = self.data_control.read('dataset/maps.json')
        obj[self.current_map] = self.navigation.tf_listener()
        self.data_control.write('dataset/maps.json', obj)
        # self.navigation.estimate_pose(obj[self.current_map])
    
    def load_map(self):
        map_name = self.ui.maps_line_3.currentText()
        subprocess.Popen(
                    f"rosrun map_server map_server {self.path}/maps/{map_name}.yaml", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid
                )
        self.ui.settings_info_2.setText(f'{map_name} is being loaded. Please wait...')
    
    def save_stations(self):
        station_name = str(int(self.current_station.objectName().split('_')[1]) - 8)
        obj = self.data_control.read('dataset/stations.json')
        obj[station_name] = self.navigation.tf_listener()
        self.data_control.write('dataset/stations.json', obj)
    
    def select_station(self):
        self.current_station.setStyleSheet(self.get_style()['inactive_style'])
        self.sender().setStyleSheet(self.get_style()['active_style'])
        self.current_station = self.sender()
    
    def set_timer(self):
        self.motion_timeout = self.ui.spin_time.value()
        self.ui.settings_info_2.setText(str(self.motion_timeout))
    
    def start_motion(self):
        if self.motion_index < len(self.stations):
            stns = self.data_control.read('dataset/stations.json')
            goal_pose = stns[self.stations[self.motion_index]]
            self.navigation.send_goal(goal_pose)
            self.check_timer = QTimer()
            self.check_timer.timeout.connect(lambda: self.goal_check(goal_pose))
            self.check_timer.start(50)
        
        if self.sender() == self.ui.start_planner:
            self.motion_index = 0
    
    def goal_check(self, goal):
        if self.navigation.goal_status(goal):
            self.check_timer.stop()
            self.motion_index += 1
            QTimer().singleShot(int(self.motion_timeout) * 1000, self.start_motion)
            # QTimer().singleShot(5000, self.start_motion)



if __name__ == "__main__":
    rospy.init_node('onder_ui', anonymous=True)
    app = QApplication(sys.argv)
    win = ui_windows()

    win.show()
    sys.exit(app.exec_())