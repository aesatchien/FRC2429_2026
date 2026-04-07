# pyqt example for teaching GUI development for FRC dashboard
# make sure to pip install pyqt6 pyqt6-tools

# print(f'Loading Modules ...', flush=True)
import os
import socket
import cv2
import numpy as np
import time, subprocess
from pathlib import Path

from PyQt6 import QtCore, QtGui, QtWidgets, uic
from PyQt6.QtCore import Qt, QTimer, QEvent
from PyQt6.QtWidgets import QGraphicsOpacityEffect
import wpimath.geometry as geo

# logical chunking of the gui's functional components
from config import WIDGET_CONFIG, CAMERA_CONFIG, SHOW_APRILTAGS, TAG_LAYOUT
import config
from nt_manager import NTManager
from camera_manager import CameraManager
from ui_updater import UIUpdater
from nt_tree import NTTreeManager  # eventually i will make this work again (my own NT tree)

# I don't think I need these here - but load ui may need them?
#from widgets.clickable_qlabel import ClickableQLabel
#from widgets.warning_label import WarningLabel


os.environ["OPENCV_LOG_LEVEL"] = "DEBUG"  # Options: INFO, WARNING, ERROR, DEBUG
print("[DEBUG] OpenCV version:", cv2.__version__)
cv2.setNumThreads(4)  # Limit OpenCV to 4 threads (use 0 to fully disable multithreading)
cv2.ocl.setUseOpenCL(True)

class SubprocessWorker(QtCore.QObject):
    """
    A worker that runs a subprocess command in a separate thread and streams its output.
    This prevents the GUI from freezing while waiting for the command to complete.
    """
    finished = QtCore.pyqtSignal()
    output_ready = QtCore.pyqtSignal(str)

    def __init__(self, command):
        super().__init__()
        self.command = command

    @QtCore.pyqtSlot()
    def run(self):
        """Executes the command and streams output line-by-line."""
        try:
            # CREATE_NO_WINDOW flag prevents a console window from popping up on Windows
            creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
            process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # Combine stdout and stderr into one stream
                text=True,
                creationflags=creationflags
            )
            # Read line by line as it comes in and emit a signal for each
            for line in iter(process.stdout.readline, ''):
                self.output_ready.emit(line.strip())
            process.stdout.close()
            process.wait()
        except Exception as e:
            self.output_ready.emit(f"Error running command: {e}")
        finally:
            self.finished.emit()

class QuestPingWorker(QtCore.QObject):
    """
    A worker that scans a range of IPs to find the Quest headset,
    skips the local machine's IP, and connects via ADB.
    """
    finished = QtCore.pyqtSignal()
    output_ready = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()

    @QtCore.pyqtSlot()
    def run(self):
        try:
            # Extract the base IP (e.g., '10.24.29') from the config
            base_ip = ".".join(config.QUESTNAV_ADB_ADDRESS.split('.')[:3])
            
            # Attempt to find our own IP on this subnet to avoid pinging ourselves
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect((f'{base_ip}.2', 1))  # connect to the radio to find local IP
                local_ip = s.getsockname()[0]
            except Exception:
                local_ip = '127.0.0.1'
            finally:
                s.close()

            self.output_ready.emit(f"Local IP detected as {local_ip}")

            found_ip = None
            creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

            # Scan range .200 through .209
            for i in range(200, 210):
                target_ip = f"{base_ip}.{i}"
                if target_ip == local_ip:
                    self.output_ready.emit(f"Skipping {target_ip} (this computer)")
                    continue

                self.output_ready.emit(f"Pinging {target_ip}...")
                # Windows: -n 1 (count), -w 500 (timeout in ms)
                # Linux/Mac: -c 1 (count), -W 1 (timeout in sec)
                ping_cmd = ['ping', '-n', '1', '-w', '500', target_ip] if os.name == 'nt' else ['ping', '-c', '1', '-W', '1', target_ip]
                
                try:
                    res = subprocess.run(ping_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, creationflags=creationflags)
                    output = res.stdout.lower()
                    # Windows ping sometimes returns 0 even if "Destination host unreachable", so we must check the text output
                    if res.returncode == 0 and b"unreachable" not in output and b"100% loss" not in output:
                        self.output_ready.emit(f"Reply received from {target_ip}!")
                        found_ip = target_ip
                        break
                except Exception as e:
                    self.output_ready.emit(f"Ping command failed: {e}")

            if found_ip:
                adb_path = os.path.join(os.path.dirname(__file__), "adb", "adb.exe")
                adb_address = f"{found_ip}:5802"
                
                # Dynamically update the config so ui_updater passthrough fix uses the found IP!
                config.QUESTNAV_ADB_ADDRESS = adb_address
                self.output_ready.emit(f"Updated config ADB target to {adb_address}")
                self.output_ready.emit(f"Attempting ADB connect...")

                process = subprocess.Popen(
                    [adb_path, "connect", adb_address],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    creationflags=creationflags
                )
                for line in iter(process.stdout.readline, ''):
                    self.output_ready.emit(line.strip())
                process.stdout.close()
                process.wait()
            else:
                self.output_ready.emit("No Quest candidate found in range 200-209.")

        except Exception as e:
            self.output_ready.emit(f"Error during quest ping process: {e}")
        finally:
            self.finished.emit()

class PiPingWorker(QtCore.QObject):
    """
    A worker that pings a list of Pi IPs to check hardware status.
    """
    finished = QtCore.pyqtSignal()
    result_ready = QtCore.pyqtSignal(str, bool)  # IP, is_alive

    def __init__(self, ips):
        super().__init__()
        self.ips = ips

    @QtCore.pyqtSlot()
    def run(self):
        creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        for ip in self.ips:
            ping_cmd = ['ping', '-n', '1', '-w', '500', ip] if os.name == 'nt' else ['ping', '-c', '1', '-W', '1', ip]
            try:
                res = subprocess.run(ping_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, creationflags=creationflags)
                output = res.stdout.lower()
                is_alive = res.returncode == 0 and b"unreachable" not in output and b"100% loss" not in output
                self.result_ready.emit(ip, is_alive)
            except Exception:
                self.result_ready.emit(ip, False)
        self.finished.emit()

#print(f'Initializing GUI ...', flush=True)

class Ui(QtWidgets.QMainWindow):
    root_dir = Path('.').absolute()
    png_dir = root_dir / 'png'
    save_dir = root_dir / 'save'

    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('layout_2026.ui', self)

        self.nt_manager = NTManager(self)
        self.camera_manager = CameraManager(self)
        self.ui_updater = UIUpdater(self)
        self.nt_tree_manager = NTTreeManager(self)
        self.ntinst = self.nt_manager.ntinst  # Keep for compatibility for now

        self.sorted_tree = None
        self.autonomous_list = []

        self.refresh_time = 50
        self.previous_frames = 0
        self.widget_dict = {}
        self.command_dict = {}

        self.ping_thread = None
        self.ping_worker = None
        self.pi_ping_thread = None
        self.pi_ping_worker = None

        self.pi_states = {}  # Tracks IP -> bool for connection edge detection

        # camera stuff - probably not needed
        self.worker = None
        # self.camera_enabled = False
        # self.thread = None

        self.robot_pixmap = QtGui.QPixmap(str(self.png_dir / 'blockhead.png'))
        self.quest_pixmap = QtGui.QPixmap(str(self.png_dir / 'quest.png'))
        self.ghost_pixmap = QtGui.QPixmap(str(self.png_dir / 'ghost.png'))
        self.target_pixmap = QtGui.QPixmap(str(self.png_dir / 'target.png'))
        self.apriltag_pixmap = QtGui.QPixmap(str(self.png_dir / 'apriltag.png'))
        
        # Create a pool of target widgets for the pose array (Original + 4 extras)
        self.target_widgets = [self.qlabel_target]
        for _ in range(4):
            w = QtWidgets.QLabel(self.qgroupbox_field)
            w.resize(41, 41)
            w.setScaledContents(True)
            w.hide()
            self.target_widgets.append(w)

        opacity_effect = QGraphicsOpacityEffect()
        opacity_effect.setOpacity(0.5)
        self.qlabel_quest.setGraphicsEffect(opacity_effect)

        ghost_effect = QGraphicsOpacityEffect()
        ghost_effect.setOpacity(0.5)
        self.qlabel_ghost.setGraphicsEffect(ghost_effect)

        # Create a border widget that will sit behind the Quest pixmap
        self.qlabel_quest_border = QtWidgets.QLabel(self.qgroupbox_field)
        self.qlabel_quest_border.setObjectName("qlabel_quest_border")
        self.qlabel_quest_border.setScaledContents(True)
        
        # Draw a custom rounded border pixmap so it can be mathematically rotated
        px_size = self.quest_pixmap.size()
        self.quest_border_pixmap = QtGui.QPixmap(px_size)
        self.quest_border_pixmap.fill(QtCore.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(self.quest_border_pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        pen = QtGui.QPen(QtGui.QColor(76, 255, 0, 160))
        pen_width = max(4, px_size.width() // 15)  # Dynamic thickness based on source image
        pen.setWidth(pen_width)
        painter.setPen(pen)
        margin = pen_width // 2
        radius = px_size.width() // 6  # Rounded corners to match the Quest logo
        painter.drawRoundedRect(margin, margin, px_size.width() - 2*margin, px_size.height() - 2*margin, radius, radius)
        painter.end()

        self.qlabel_quest_border.hide()
        self.qlabel_quest.raise_()
        self.qlabel_robot.raise_()

        # Build the runtime dictionaries from the static config
        self.widget_dict = self.build_widget_dict()
        self.camera_dict = self.build_camera_dict()
        # print(self.widget_dict)

        # set the colors for the warning labels
        self.qlabel_pdh_voltage_monitor.update_settings(min_val=8, max_val=12, red_high=False, display_float=True)
        self.qlabel_pdh_current_monitor.update_settings(min_val=60, max_val=160, red_high=True, display_float=False)
        self.qlabel_questnav_dtap_count_monitor.update_settings(min_val=0, max_val=10, red_high=True, display_float=False)

        # Publishers for key presses
        self.keys_pressed_pub = self.ntinst.getIntegerArrayTopic("/SmartDashboard/keys_pressed").publish()
        self.key_pressed_pub = self.ntinst.getIntegerTopic("/SmartDashboard/key_pressed").publish()

        self.initialize_widgets()
        self.initialize_apriltags()

        # Connections for the NT Tree are now managed by the NTTreeManager
        self.qaction_show_hide.triggered.connect(self.nt_tree_manager.toggle_network_tables)
        self.qaction_refresh.triggered.connect(self.nt_tree_manager.refresh_tree)
        self.qlistwidget_commands.clicked.connect(self.nt_tree_manager.command_list_clicked)
        self.qcombobox_nt_keys.currentTextChanged.connect(self.nt_tree_manager.update_selected_key)
        self.qt_tree_widget_nt.clicked.connect(self.nt_tree_manager.qt_tree_widget_nt_clicked)
        self.qt_text_entry_filter.textChanged.connect(self.nt_tree_manager.filter_nt_keys_combo)
        self.qt_button_set_key.clicked.connect(self.nt_tree_manager.update_key)

        # Other UI connections
        # CRITICAL: We MUST use textActivated and NOT currentTextChanged for combo boxes that sync with the robot.
        # textActivated only fires when a HUMAN physically clicks a choice in the GUI.
        # If we use currentTextChanged, populating the list on startup or receiving a new selection from the 
        # robot will fire the signal, causing the GUI to instantly write its own value back to the robot, 
        # overwriting the robot's intended state and creating a race condition/feedback loop.
        self.qcombobox_autonomous_routines.textActivated.connect(self.update_routines)
        self.qt_text_entry_filter.installEventFilter(self)
        self.qt_text_new_value.installEventFilter(self)

        # Same as above: textActivated prevents the GUI from echoing the robot's initial delay broadcast 
        # back to the robot as a user-requested change.
        self.qcombobox_auto_delay.textActivated.connect(self.update_auto_delay)
        self.qt_button_swap_sim.clicked.connect(self.nt_manager.increment_server)
        self.qt_button_reconnect.clicked.connect(self.nt_manager.reconnect)
        self.qt_button_camera_enable.clicked.connect(self.camera_manager.toggle_camera_thread)
        self.qt_button_ping_quest.clicked.connect(self.ping_quest)
        self.qt_button_quest_test_dt.clicked.connect(self.ui_updater._test_questnav_dtap)

        self.qt_tree_widget_nt.hide()

        self.keys_currently_pressed = []
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.show()

        self.counter = 1
        self.previous_time = time.time()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ui_updater.update_widgets)
        self.timer.start(self.refresh_time)

        self.pi_ping_timer = QTimer(self)
        self.pi_ping_timer.timeout.connect(self.start_pi_ping)
        self.pi_ping_timer.start(int(getattr(config, 'PI_PING_INTERVAL_S', 5.0) * 1000))
        self.start_pi_ping()  # Trigger initial check immediately

    def build_widget_dict(self):
        """Builds the runtime widget dictionary from the static configuration."""
        widget_dict = {}
        for key, config in WIDGET_CONFIG.items():
            new_entry = config.copy()
            widget_name = config.get('widget_name')
            if widget_name:
                new_entry['widget'] = getattr(self, widget_name, None)
            
            new_entry['last_value'] = None
            # Determine Topic Type based on update_style
            style = config.get('update_style')
            nt_topic = config.get('nt_topic')
            
            if nt_topic:
                if style == 'indicator':
                    new_entry['subscriber'] = self.ntinst.getBooleanTopic(nt_topic).subscribe(False)
                elif style in ['lcd', 'monitor', 'time', 'hub']:
                    new_entry['subscriber'] = self.ntinst.getDoubleTopic(nt_topic).subscribe(0.0)
                elif style == 'pose':
                    new_entry['subscriber'] = self.ntinst.getStructTopic(nt_topic, geo.Pose2d).subscribe(geo.Pose2d())
                elif style == 'pose_array':
                    new_entry['subscriber'] = self.ntinst.getStructArrayTopic(nt_topic, geo.Pose2d).subscribe([])
                    if key == 'target_pose':
                        new_entry['widgets'] = self.target_widgets
                elif style == 'combo':
                    new_entry['subscriber'] = self.ntinst.getStringArrayTopic(nt_topic).subscribe([])
                elif style in ['position', 'active_hub']:
                    new_entry['subscriber'] = self.ntinst.getStringTopic(nt_topic).subscribe("")
                elif style == 'numeric_combo':
                    new_entry['subscriber'] = self.ntinst.getDoubleTopic(nt_topic).subscribe(0.0)
                    new_entry['publisher'] = self.ntinst.getDoubleTopic(nt_topic).publish()
                else:
                    print(f'[{self.ui_updater.get_elapsed_time():.1f}] cannot determine subscriber type for - check style!')

            command_topic = config.get('command_topic')
            if command_topic:
                new_entry['publisher'] = self.ntinst.getBooleanTopic(command_topic).publish()
                new_entry['command_subscriber'] = self.ntinst.getBooleanTopic(command_topic).subscribe(False)

            selected_topic = config.get('selected_topic')
            if selected_topic:
                new_entry['selected_subscriber'] = self.ntinst.getStringTopic(selected_topic).subscribe("")
                new_entry['last_selected_value'] = None
                new_entry['selected_publisher'] = self.ntinst.getStringTopic(selected_topic).publish()
                # print(f'{key} has selected topic: {selected_topic} with value {new_entry[selected_subscriber].get()'}

            active_topic = config.get('active_topic')
            if active_topic:
                new_entry['active_subscriber'] = self.ntinst.getStringTopic(active_topic).subscribe("")

            visible_topic = config.get('visible_topic')  # sometimes we want to hide things
            if visible_topic:
                new_entry['visible_subscriber'] = self.ntinst.getBooleanTopic(visible_topic).subscribe(False)
                new_entry['last_visible_value'] = None

            widget_dict[key] = new_entry
        # print(widget_dict)
        return widget_dict

    def build_camera_dict(self):
        """Builds the runtime camera dictionary from the static configuration."""
        camera_dict = {}
        for key, config in CAMERA_CONFIG.items():
            new_entry = config.copy()
            
            indicator_name = config.get('HEARTBEAT_INDICATOR_NAME')
            if indicator_name:
                new_entry['INDICATOR'] = getattr(self, indicator_name, None)

            # update the text on the target indicators when we start - now we don't have to edit UI if we change names
            target_indicator_name = config.get('TARGET_INDICATOR_NAME')
            if target_indicator_name:
                target_widget = getattr(self, target_indicator_name, None)
                if target_widget:
                    nickname = config.get('NICKNAME', key)
                    if target_widget.width() < 100:
                        nickname = nickname.replace(' ', '\n')
                    target_widget.setText(nickname)

            framecount_topic = config.get('FRAMECOUNT_TOPIC')
            if framecount_topic:
                new_entry['IS_ALIVE'] = False
                new_entry['RECONNECTION_COUNT'] = 0
                new_entry['last_is_alive'] = None
                new_entry['last_connections'] = None
                new_entry['FRAMECOUNT_SUB'] = self.ntinst.getDoubleTopic(framecount_topic).subscribe(-1)

            connections_topic = config.get('CONNECTIONS_TOPIC')
            if connections_topic:
                new_entry['CONNECTIONS_SUB'] = self.ntinst.getDoubleTopic(connections_topic).subscribe(0)

            camera_dict[key] = new_entry
        return camera_dict

    def update_routines(self, text):
        pub = self.widget_dict['qcombobox_autonomous_routines'].get('selected_publisher')
        if pub:
            pub.set(text)
        self.ntinst.flush()

    def update_auto_delay(self, text):
        """Publishes the selected auto delay value to NetworkTables."""
        pub = self.widget_dict['qcombobox_auto_delay'].get('publisher')
        if pub:
            try:
                value = float(text)
                pub.set(value)
            except (ValueError, TypeError):
                print(f'[{self.ui_updater.get_elapsed_time():.1f}] Invalid value for auto delay: {text}')

    def label_click(self, label):
        props = self.widget_dict[label]
        pub = props.get('publisher')
        sub = props.get('command_subscriber')
        
        if pub and sub:
            # Check if the topic has data. If time is 0, it means no value has been set (by robot or us).
            atomic_val = sub.getAtomic()
            if atomic_val.time == 0:
                print(f'[{self.ui_updater.get_elapsed_time():.1f}] Warning: {label} clicked but NT topic {sub.getTopic().getName()} has no value (Robot not connected?).', flush=True)
                return

            # Toggle based on current state of the command topic
            toggled_state = not atomic_val.value
            print(f'[{self.ui_updater.get_elapsed_time():.1f}] You clicked {label}. Firing command ...', flush=True)
            pub.set(toggled_state)
        else:
            print(f'[{self.ui_updater.get_elapsed_time():.1f}] Warning: {label} clicked but NT Publisher or Subscriber is missing.', flush=True)

    def ping_quest(self):
        """Pings a range of IPs to find the Quest ADB server and connect without freezing."""
        # Prevent running multiple pings at once
        if self.ping_thread and self.ping_thread.isRunning():
            self.qt_text_status.appendPlainText("Search already in progress...")
            return

        # Dynamically determine the base IP from config
        base_ip = ".".join(config.QUESTNAV_ADB_ADDRESS.split('.')[:3])
        self.qt_text_status.appendPlainText(f"[{self.ui_updater.get_elapsed_time():.1f}] Searching for QuestNav in {base_ip}.200-209 range...")

        # 1. Create a thread and a worker
        self.ping_thread = QtCore.QThread()
        self.ping_worker = QuestPingWorker()

        # 2. Move worker to the thread
        self.ping_worker.moveToThread(self.ping_thread)

        # 3. Connect signals and slots
        self.ping_worker.output_ready.connect(self.qt_text_status.appendPlainText)
        self.ping_thread.started.connect(self.ping_worker.run)
        self.ping_worker.finished.connect(self.ping_thread.quit)
        
        # 4. Start the thread
        self.ping_thread.start()

    def start_pi_ping(self):
        """Fires off a background worker to ping all unique IPs in the camera config."""
        if self.pi_ping_thread and self.pi_ping_thread.isRunning():
            return  # Skip if the last scan is still somehow running
            
        # Extract unique IPs
        ips = set(props['IP'] for props in self.camera_dict.values() if 'IP' in props)
        if not ips:
            return

        self.pi_ping_thread = QtCore.QThread()
        self.pi_ping_worker = PiPingWorker(list(ips))
        self.pi_ping_worker.moveToThread(self.pi_ping_thread)
        
        self.pi_ping_worker.result_ready.connect(self.handle_pi_ping_result)
        self.pi_ping_thread.started.connect(self.pi_ping_worker.run)
        self.pi_ping_worker.finished.connect(self.pi_ping_thread.quit)
        
        self.pi_ping_thread.start()

    def handle_pi_ping_result(self, ip, is_alive):
        """Processes the ping results and cross-references with camera script status."""
        prev_state = self.pi_states.get(ip)
        self.pi_states[ip] = is_alive
        elapsed = self.ui_updater.get_elapsed_time()
        
        # Pass the hardware alive state to the camera dict for the UI Updater
        for props in self.camera_dict.values():
            if props.get('IP') == ip:
                props['PI_ALIVE'] = is_alive
        
        if prev_state is None:
            status = "ONLINE" if is_alive else "OFFLINE"
            self.qt_text_status.appendPlainText(f"[{elapsed:.1f}] Pi at {ip} is {status} (Initial Check).")
        elif is_alive and not prev_state:
            self.qt_text_status.appendPlainText(f"[{elapsed:.1f}] Pi at {ip} RECONNECTED.")
        elif not is_alive and prev_state:
            self.qt_text_status.appendPlainText(f"[{elapsed:.1f}] WARNING: Pi at {ip} DROPPED CONNECTION!")
            
        # If Pi is responding, cross-check to see if the vision script crashed
        if is_alive and elapsed > 5.0:  # Wait 5 seconds after GUI boot before issuing crash warnings
            dead_cams = [props.get('NICKNAME', name) for name, props in self.camera_dict.items() 
                         if props.get('IP') == ip and 'IS_ALIVE' in props and not props['IS_ALIVE']]
            
            warn_key = f"pi_{ip}_cam_warn"
            if dead_cams:
                if not getattr(self, warn_key, False):
                    cams_str = ", ".join(dead_cams)
                    self.qt_text_status.appendPlainText(f"[{elapsed:.1f}] CRITICAL: Pi {ip} responding, but cameras DEAD ({cams_str})!")
                    setattr(self, warn_key, True)
            else:
                if getattr(self, warn_key, False):
                    self.qt_text_status.appendPlainText(f"[{elapsed:.1f}] Cameras on Pi {ip} have recovered.")
                    setattr(self, warn_key, False)

    def initialize_widgets(self):
        """Connects widget signals and populates the camera combobox."""
        # Connect clickable label signals
        for key, widget_props in self.widget_dict.items():
            if widget_props.get('publisher') and widget_props.get('widget'):
                widget_props['widget'].clicked.connect(lambda label=key: self.label_click(label))
        
        # Populate camera combobox
        seen_urls = set()
        for key, props in self.camera_dict.items():
            url = props.get('URL')
            if url and url not in seen_urls:
                self.qcombobox_cameras.addItem(key)
                seen_urls.add(url)

        # Populate auto delay combobox with values 0-10
        # Block signals to prevent this from firing a currentTextChanged event on startup
        delay_options = [str(i) for i in range(11)]
        self.qcombobox_auto_delay.blockSignals(True)
        self.qcombobox_auto_delay.addItems(delay_options)
        self.qcombobox_auto_delay.blockSignals(False)

    def initialize_apriltags(self):
        """Loads the AprilTag layout and places static tag widgets on the field."""
        if not SHOW_APRILTAGS:
            return

        try:
            layout = TAG_LAYOUT
        except Exception as e:
            print(f"[{self.ui_updater.get_elapsed_time():.1f}] Error loading AprilTag layout: {e}")
            return

        field_width = self.qgroupbox_field.width()
        field_height = self.qgroupbox_field.height()

        scale_x = field_width / config.REF_IMG_WIDTH
        scale_y = field_height / config.REF_IMG_HEIGHT
        
        # Iterate through all tags in the layout
        for tag in layout.getTags():
            pose = tag.pose.toPose2d()
            x, y, rot = pose.X(), pose.Y(), pose.rotation().degrees()

            # Create a new label for the tag
            label = QtWidgets.QLabel(self.qgroupbox_field)
            label.setScaledContents(True)
            label.setToolTip(f"ID: {tag.ID}")
            label.show()

            # Rotate and scale the pixmap (using target.png as the tag icon)
            base_size = 20 
            pixmap_rotated = self.apriltag_pixmap.transformed(QtGui.QTransform().rotate(90 - rot), QtCore.Qt.TransformationMode.SmoothTransformation)
            new_size = int(base_size * (1 + 0.41 * np.abs(np.sin(2 * rot * np.pi / 180.0))))
            label.resize(new_size, new_size)
            label.setPixmap(pixmap_rotated)

            # Convert field coordinates (meters) to widget coordinates (pixels)
            # Map Field X (0 to FIELD_LENGTH) to Pixel X
            field_x_frac = x / config.FIELD_LENGTH
            ref_px_x = config.REF_BL_PX[0] + field_x_frac * (config.REF_TR_PX[0] - config.REF_BL_PX[0])
            widget_x = int(-new_size / 2 + ref_px_x * scale_x)

            # Map Field Y (0 to FIELD_WIDTH) to Pixel Y
            field_y_frac = y / config.FIELD_WIDTH
            ref_px_y = config.REF_BL_PX[1] + field_y_frac * (config.REF_TR_PX[1] - config.REF_BL_PX[1])
            widget_y = int(-new_size / 2 + ref_px_y * scale_y)

            label.move(widget_x, widget_y)

    def keyPressEvent(self, event):
        if event.key() not in self.keys_currently_pressed:
            self.keys_currently_pressed.append(event.key())
        self.keys_pressed_pub.set(self.keys_currently_pressed)
        self.key_pressed_pub.set(event.key())

    def keyReleaseEvent(self, event):
        try:
            while event.key() in self.keys_currently_pressed:
                self.keys_currently_pressed.remove(event.key())
        except ValueError:
            pass
        self.key_pressed_pub.set(-999)
        self.keys_pressed_pub.set(self.keys_currently_pressed)

    def focusOutEvent(self, event):
        self.keys_currently_pressed = []
        self.keys_pressed_pub.set(self.keys_currently_pressed)

    def eventFilter(self, obj, event):
        if (obj is self.qt_text_entry_filter or obj is self.qt_text_new_value) and event.type() == QEvent.Type.KeyPress:
            if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                return True
        return super().eventFilter(obj, event)

    def convert_cv_qt(self, cv_img, qlabel):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        p = convert_to_Qt_format.scaled(qlabel.width(), qlabel.height(), Qt.AspectRatioMode.KeepAspectRatio)
        return QtGui.QPixmap.fromImage(p)