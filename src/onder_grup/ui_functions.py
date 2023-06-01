from main import *
import math
from PyQt5.QtCore import QPropertyAnimation, QEasingCurve

from rviz import bindings as rviz

class UIFunctions(ui_windows):

    def animations(self,maxWidth, widget):
        # GET WIDTH
        width = widget.height()
        print(width)
        maxExtend = maxWidth
        standard = 0

        # SET MAX WIDTH
        if width == 0:
            widthExtended = maxExtend
        else:
            widthExtended = standard

        # ANIMATION

        self.animation = QPropertyAnimation(widget, b"maximumHeight")
        self.animation.setDuration(500)
        self.animation.setStartValue(width)
        self.animation.setEndValue(widthExtended)
        self.animation.setEasingCurve(QEasingCurve.InOutQuart)
        self.animation.start()
    
    def print_mouse_position(self):
        margin = 50
        frame_dx = self.ui.right_frame.geometry().x() + margin
        frame_dy = self.ui.vel_control_frame.geometry().y() + self.ui.top_frame.geometry().height() + margin - 30
        mouse_pos = self.mapFromGlobal(self.cursor().pos())
        button_geo = self.ui.joy_inside.geometry()
        limit_geo = self.ui.joy_out.geometry()
        cx = limit_geo.x() + limit_geo.width()/2
        cy = limit_geo.y() + limit_geo.height()/2
        d = math.sqrt((mouse_pos.x() - frame_dx - cx)**2 + (mouse_pos.y() - frame_dy - cy)**2)

        if d <= limit_geo.height()/2:
            x = int(mouse_pos.x() - frame_dx -button_geo.width()/2)
            y = int(mouse_pos.y() - frame_dy -button_geo.height()/2)
            max_linear_vel = limit_geo.x() + limit_geo.width() - cx
            max_angular_vel = limit_geo.y() + limit_geo.height() - cy
            self.ang_vel = ((x + button_geo.width()/2) - cx) * self.angular_vel / max_linear_vel
            self.lin_vel = ((y + button_geo.height()/2) - cy) * self.linear_vel / max_angular_vel
            UIFunctions.set_velocity(self, abs(round(self.lin_vel, 2)))
            self.ui.joy_inside.move(x, y)
            self.vel_pub.send_vel(self.lin_vel, self.ang_vel)
    
    def set_velocity(self, value):
        styleSheet = """
        	QFrame#vel_out{	background-color: qconicalgradient(cx:0.5, cy:0.5, angle:230, stop:{STOP_1} rgba(88, 216, 244, 255), stop:{STOP_2} rgba(255, 255, 255, 3));
                border-radius: 105px;}
        """
        err = 0.22
        max_value = self.linear_vel - self.linear_vel * err
        value = value * max_value / self.linear_vel
        progress = (self.linear_vel-value) / self.linear_vel

        stop_1 = str(progress)
        stop_2 = str(progress - 0.05)

        newStyleSheet = styleSheet.replace("{STOP_1}", stop_1).replace("{STOP_2}",stop_2)

        self.ui.vel_out.setStyleSheet(newStyleSheet)
        self.ui.vel_label.setText(str(abs(round(self.lin_vel, 2))))
    
    def add_rviz(self):
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, f"{self.path}/src/onder_grup/rur_navigation.rviz" )
        self.frame.load( config )
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        # layout = QVBoxLayout()
        self.ui.gridLayout.addWidget(self.frame, 0,0,1,1)
