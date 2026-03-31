# This file contains the UIUpdater class, which is responsible for updating the widgets in the main UI.

import time
import numpy as np
import os
import subprocess
import wpimath.geometry as geo
import ntcore
import config
from PyQt6 import QtGui, QtCore

class UIUpdater:
    # Define styles as class constants to avoid magic strings
    STYLE_ON = "border: 7px; border-radius: 7px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"
    STYLE_OFF = "border: 7px; border-radius: 7px; background-color:rgb(220, 0, 0); color:rgb(200, 200, 200);"
    STYLE_HIGH = "border: 7px; border-radius: 15px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"
    STYLE_LOW = "border: 7px; border-radius: 15px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"
    STYLE_FLASH_ON = "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 0); color:rgb(255, 255, 255);"
    STYLE_FLASH_OFF = "border: 7px; border-radius: 7px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"
    STYLE_DISCONNECTED = "border: 7px; border-radius: 7px; background-color:rgb(180, 180, 180); color:rgb(0, 0, 0);"

    def __init__(self, ui):
        self.ui = ui
        self.drive_pose = geo.Pose2d()  # Store pose for other calculations
        self.flash_on = False
        self.dtap_start_time = 0.0
        self.dtap_last_fix_time = 0.0
        self.dtap_retries = 0
        self.start_time = time.time()

        # Map update styles from config to specific update functions
        self.updaters = {
            'indicator': self._update_indicator,
            'combo': self._update_combo,
            'time': self._update_time,
            'monitor': self._update_monitor,
            'lcd': self._update_lcd,
            'position': self._update_position,  # Legacy from 2024
            'active_hub': self._update_active_hub,
            'numeric_combo': self._update_numeric_combo,
            'hub': self._update_hub,  # Legacy from 2023
        }
        
    def get_elapsed_time(self):
        """Returns the number of seconds since the GUI started."""
        return time.time() - self.start_time

    def update_widgets(self):
        """Main update loop. Orchestrates calls to specialized update functions."""
        self._update_connection_status()  # check if NT is connected
        
        # Calculate flash state once per cycle
        self.flash_on = self.ui.counter % 30 < 15

        # Functions with dependencies are called in a specific order
        self._update_pose_and_field()  # Must be called first to get latest pose
        self._update_camera_indicators()  # Check camera connection states
        self._update_shot_calculations()  # Depends on pose data
        self._check_questnav_passthrough_issue()

        # Use the dispatcher for all other standard widgets based on their update style
        for widget_props in self.ui.widget_dict.values():
            update_style = widget_props.get('update_style')
            if update_style in self.updaters:
                self.updaters[update_style](widget_props)  # call the appropriate update method

        self._update_command_list()  # autonomous commands
        self._update_fps_counter()  # status bar updates

        self.ui.counter += 1

    # --------------------------------------------------------------------------
    # Specialized Update Functions (Called from the main update_widgets loop)
    # --------------------------------------------------------------------------

    def _update_connection_status(self):
        """Updates the NT connection status indicator and handles disconnect cleanup."""
        is_connected = self.ui.ntinst.isConnected()
        widget = self.ui.widget_dict['qlabel_nt_connected']['widget']
        style = self.STYLE_ON if is_connected else self.STYLE_DISCONNECTED
        widget.setStyleSheet(style)

        # WIPE THE SLATE CLEAN ON DISCONNECT
        # Detect transition from connected to disconnected
        was_connected = getattr(self, '_last_connection_state', True)
        if not is_connected and was_connected:
            print(f"[{self.get_elapsed_time():.1f}] Network disconnected. Wiping chooser state to prevent stale data on reconnect.")
            for props in self.ui.widget_dict.values():
                if props.get('update_style') == 'combo':
                    # 1. Clear the UI to give visual feedback that the options are no longer valid.
                    combo_widget = props.get('widget')
                    if combo_widget:
                        combo_widget.blockSignals(True)
                        combo_widget.clear()
                        combo_widget.blockSignals(False)
                    # 2. Reset the internal state caches to prevent stale comparisons on reconnect.
                    props['last_value'] = []
                    props['last_selected_value'] = ""

                    # 3. CRITICAL: Close the publisher to erase its cached value from the NT4 client's memory.
                    # This is the key step that prevents the GUI from forcing its old selection on a newly rebooted robot.
                    pub = props.get('selected_publisher')
                    if pub:
                        pub.close()
                        props['selected_publisher'] = None # Mark as closed
                    
        elif is_connected and not was_connected:
            print(f"[{self.get_elapsed_time():.1f}] Network reconnected. Re-creating chooser publisher to allow user input.")
            # 4. Re-create the publisher on reconnect so the user can make selections again.
            # The GUI now has no cached value to send and will wait for the robot's broadcast.
            props = self.ui.widget_dict.get('qcombobox_autonomous_routines')
            if props and props.get('selected_topic'):
                props['selected_publisher'] = self.ui.ntinst.getStringTopic(props['selected_topic']).publish()

        self._last_connection_state = is_connected

    def _check_questnav_passthrough_issue(self):
        """Monitors QuestNav passthrough state and restarts app via adb if stuck."""
        dtap_props = self.ui.widget_dict.get('qlabel_questnav_dtap_indicator')
        if not dtap_props:
            return

        sub = dtap_props.get('subscriber')
        if not sub:
            return

        is_in_passthrough = sub.get()
        current_time = self.get_elapsed_time()

        if is_in_passthrough:
            # Start the timer if it just became True
            if self.dtap_start_time == 0.0:
                self.dtap_start_time = current_time
                print(f"[{current_time:.1f}] We have detected a QuestNav passthru condition.", flush=True)

            # If True for more than 1 second
            if (current_time - self.dtap_start_time) > 1.0:
                # Check cooldown of 2 second since last fix
                if (current_time - self.dtap_last_fix_time) > 2.0:
                    if self.dtap_retries < config.QUESTNAV_ADB_MAX_RETRIES:
                        self.dtap_retries += 1
                        print(f"[{current_time:.1f}] QuestNav has been in passthrough for > 1s. Attempting ADB fix ({self.dtap_retries}/{config.QUESTNAV_ADB_MAX_RETRIES}).", flush=True)
                        
                        try:
                            adb_path = os.path.join(os.path.dirname(__file__), "adb", "adb.exe")
                            cmd = [adb_path, "-s", config.QUESTNAV_ADB_ADDRESS, "shell", "am", "start", "-n", "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity"]
                            subprocess.Popen(cmd)
                            print(f"[{current_time:.1f}] ADB command sent. Waiting for cooldown...", flush=True)
                        except Exception as e:
                            print(f"[{current_time:.1f}] Failed to execute QuestNav ADB fix: {e}", flush=True)
                    elif self.dtap_retries == config.QUESTNAV_ADB_MAX_RETRIES:
                        print(f"[{current_time:.1f}] QuestNav ADB fix max retries ({config.QUESTNAV_ADB_MAX_RETRIES}) reached. Giving up.", flush=True)
                        self.dtap_retries += 1  # Increment once more so we don't spam the 'Giving up' message

                    self.dtap_last_fix_time = current_time
                    self.dtap_start_time = current_time  # Reset the start time so it requires another full second to trigger again if it remains True
        else:
            self.dtap_start_time = 0.0
            if self.dtap_retries > 0:
                print(f"[{current_time:.1f}] QuestNav passthru resolved.", flush=True)
                self.dtap_retries = 0

    def _update_pose_and_field(self):
        """Updates the robot and quest pose on the field graphic."""
        drive_props = self.ui.widget_dict['drive_pose']
        drive_pose_sub = drive_props.get('subscriber')
        if not drive_pose_sub:
            return # Can't do anything without the robot pose

        self.drive_pose = drive_pose_sub.get()
        quest_props = self.ui.widget_dict['quest_pose']
        quest_pose_sub = quest_props.get('subscriber')
        self.quest_pose = quest_pose_sub.get()
        field_dims = (self.ui.qgroupbox_field.width(), self.ui.qgroupbox_field.height())

        # Update Robot Pose if changed
        if self.drive_pose != drive_props.get('last_value'):
            drive_props['last_value'] = self.drive_pose
            self.ui.qlabel_pose_indicator.setText(self._format_pose_string("POSE", self.drive_pose))
            self._update_pose_widget(self.ui.qlabel_robot, self.ui.robot_pixmap, self.drive_pose, field_dims)

        # Update Quest Pose if changed
        if self.quest_pose != quest_props.get('last_value'):
            quest_props['last_value'] = self.quest_pose
            self.ui.qlabel_quest_pose_indicator.setText(self._format_pose_string("QUEST POSE", self.quest_pose))
            self._update_pose_widget(self.ui.qlabel_quest, self.ui.quest_pixmap, self.quest_pose, field_dims)

        # Update the Quest border to be visible and positioned around the Quest pixmap
        border_widget = self.ui.qlabel_quest_border
        quest_widget = self.ui.qlabel_quest
        if getattr(config, 'ADD_QUEST_BORDER', False):
            offset = -2  # 0 exactly matches bounding box. Use -1 or -2 to hug tighter if the PNG has transparent margins.
            quest_geom = quest_widget.geometry()
            border_widget.setGeometry(
                quest_geom.x() - offset,
                quest_geom.y() - offset,
                quest_geom.width() + (2 * offset),
                quest_geom.height() + (2 * offset)
            )
            if not border_widget.isVisible():
                border_widget.show()
        elif border_widget.isVisible():
            border_widget.hide()

        # Update Ghost Pose
        ghost_props = self.ui.widget_dict['ghost_pose']
        ghost_sub = ghost_props.get('subscriber')
        visible_sub = ghost_props.get('visible_subscriber')

        is_visible = (visible_sub.get() if visible_sub else False) and getattr(config, 'DRAW_GHOST_POSE', True)

        if is_visible and ghost_sub:
            ghost_pose = ghost_sub.get()
            if ghost_pose != ghost_props.get('last_value') or is_visible != ghost_props.get('last_visible_value'):
                ghost_props['last_value'] = ghost_pose
                self.ui.qlabel_ghost.show()
                self._update_pose_widget(self.ui.qlabel_ghost, self.ui.ghost_pixmap, ghost_pose, field_dims)
        else:
            if self.ui.qlabel_ghost.isVisible():
                self.ui.qlabel_ghost.hide()

        ghost_props['last_visible_value'] = is_visible

        # Update Target Pose (Array)
        target_props = self.ui.widget_dict['target_pose']
        target_sub = target_props.get('subscriber')
        visible_sub = target_props.get('visible_subscriber')
        target_widgets = target_props.get('widgets', [self.ui.qlabel_target])

        is_visible = visible_sub.get() if visible_sub else False

        if is_visible and target_sub:
            target_poses = target_sub.get() # List of Pose2d
            if target_poses != target_props.get('last_value') or is_visible != target_props.get('last_visible_value'):
                target_props['last_value'] = target_poses
                for i, widget in enumerate(target_widgets):
                    if i < len(target_poses):
                        widget.show()
                        self._update_pose_widget(widget, self.ui.target_pixmap, target_poses[i], field_dims)
                    else:
                        widget.hide()
        else:
            if target_props.get('last_visible_value') != False:
                for widget in target_widgets:
                    if widget.isVisible():
                        widget.hide()

        target_props['last_visible_value'] = is_visible

    def _update_camera_indicators(self):
        """Updates the status indicators for all cameras.
        We check the NT timestamp from the camera framecount from the camera"""
        # 1.5 seconds allowed delay (500,000 microseconds) - we update the framecount once per second
        allowed_delay_us = 2500000
        now = ntcore._now()

        for cam_props in self.ui.camera_dict.values():
            if 'FRAMECOUNT_SUB' in cam_props:
                # Check how long ago the camera updated its _frames topic
                is_alive = (now - cam_props['FRAMECOUNT_SUB'].getAtomic().time) < allowed_delay_us

                # Check for rising edge (False -> True) to count reconnections
                if is_alive and not cam_props['IS_ALIVE']:
                    cam_props['RECONNECTION_COUNT'] += 1

                cam_props['IS_ALIVE'] = is_alive

                # if we ever start serving _connections from the robot, then this will override the local count
                connections = cam_props['RECONNECTION_COUNT']
                if 'CONNECTIONS_SUB' in cam_props:
                    atomic_conn = cam_props['CONNECTIONS_SUB'].getAtomic()
                    if atomic_conn.time != 0:
                        connections = int(atomic_conn.value)

                # Only update widget if visual state changed
                if is_alive == cam_props.get('last_is_alive') and connections == cam_props.get('last_connections'):
                    continue
                cam_props['last_is_alive'] = is_alive
                cam_props['last_connections'] = connections

                style = self.STYLE_ON if is_alive else self.STYLE_OFF
                indicator = cam_props.get('INDICATOR')  # switch to safe access since we can have cameras w/o heartbeat indicators
                if indicator:
                    indicator.setStyleSheet(style)
                    indicator.setText(f'{cam_props["NICKNAME"]}: {connections:2d}')

    def _update_shot_calculations(self):
        """ Calculates and displays shot distance and angle to speaker - legacy from 2024. """
        # alliance_sub = self.ui.widget_dict['qlabel_alliance_indicator'].get('subscriber')
        # if not alliance_sub:
        #     return
        #
        # is_red_alliance = alliance_sub.get()
        # k_speaker = [16.5, 5.555, 0] if is_red_alliance else [0, 5.55, 180]
        # speaker_coords = (16.54, 5.56) if is_red_alliance else (0, 5.56)
        #
        # translation_origin_to_speaker = geo.Translation2d(k_speaker[0], k_speaker[1])
        # translation_origin_to_robot = self.drive_pose.translation()
        # translation_robot_to_speaker = translation_origin_to_speaker - translation_origin_to_robot
        # desired_angle = translation_robot_to_speaker.angle().rotateBy(geo.Rotation2d(np.radians(180)))
        # angle_to_speaker = self.drive_pose.rotation().degrees() - desired_angle.degrees()
        # shot_distance = np.sqrt((speaker_coords[0] - self.drive_pose.X())**2 + (speaker_coords[1] - self.drive_pose.Y())**2)
        #
        # best_distance, dist_tolerance, angle_tolerance = 1.7, 0.4, 10
        # shot_style = self.STYLE_OFF
        # if best_distance - dist_tolerance < shot_distance < best_distance + dist_tolerance:
        #     grey_val = int(225 * abs(best_distance - shot_distance))
        #     in_angle = abs(angle_to_speaker) < angle_tolerance
        #     text_color = '(0,0,0)' if self.ui.counter % 10 < 5 and in_angle else '(255,255,255)'
        #     border_color = 'solid blue' if self.ui.counter % 10 < 5 and in_angle else 'solid black'
        #     border_size = 6 if in_angle else 8
        #     shot_style = f"border: {border_size}px {border_color}; border-radius: 7px; background-color:rgb({grey_val}, {int(225-grey_val)}, {grey_val}); color:rgb{text_color};"
        # elif shot_distance >= best_distance + dist_tolerance:
        #     shot_style = self.STYLE_DISCONNECTED

        shot_style = self.STYLE_DISCONNECTED
        shot_distance_sub = self.ui.widget_dict['qlabel_shot_distance'].get('subscriber')
        shot_distance = shot_distance_sub.get() if shot_distance_sub else 0
        self.ui.qlabel_shot_distance.setText(f'SHOT DIST\n{shot_distance:.2f}m')
        self.ui.qlabel_shot_distance.setStyleSheet(shot_style)

    def _format_pose_string(self, label, pose):
        """Formats a pose array into a display string with appropriate padding."""
        x, y, rot = pose.X(), pose.Y(), pose.rotation().degrees()
        x_pad = 1 if x < 10 else 0
        # This logic determines padding based on the number of digits in the angle
        theta_pad = sum([1 for t in [0, 100, 10] if abs(rot) < t])
        return f'{label}\n{" " * x_pad}{x:>5.2f}m {y:>4.2f}m {" " * theta_pad}{rot:>4.0f}°'

    def _update_pose_widget(self, widget, pixmap, pose, field_dims):
        """Updates a QLabel on the field graphic with a rotated pixmap and new position."""
        x, y, rot = pose.X(), pose.Y(), pose.rotation().degrees()
        width, height = field_dims

        pixmap_rotated = pixmap.transformed(QtGui.QTransform().rotate(90 - rot), QtCore.Qt.TransformationMode.SmoothTransformation)
        # This formula creates a pulsating effect as the robot rotates
        new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * rot * np.pi / 180.0))))

        widget.resize(new_size, new_size)
        widget.setPixmap(pixmap_rotated)

        # Convert pose (meters) to widget coordinates (pixels)
        # Calculate scale factors based on current widget size vs reference image size
        scale_x = width / config.REF_IMG_WIDTH
        scale_y = height / config.REF_IMG_HEIGHT

        # Map Field X (0 to FIELD_LENGTH) to Pixel X
        field_x_frac = x / config.FIELD_LENGTH
        ref_px_x = config.REF_BL_PX[0] + field_x_frac * (config.REF_TR_PX[0] - config.REF_BL_PX[0])
        widget_x = int(-new_size / 2 + ref_px_x * scale_x)

        # Map Field Y (0 to FIELD_WIDTH) to Pixel Y
        field_y_frac = y / config.FIELD_WIDTH
        ref_px_y = config.REF_BL_PX[1] + field_y_frac * (config.REF_TR_PX[1] - config.REF_BL_PX[1])
        widget_y = int(-new_size / 2 + ref_px_y * scale_y)

        widget.move(widget_x, widget_y)

    def _update_command_list(self):
        """Updates the background color of the command list widget in the NT tree - which is currently broken"""
        green, white = QtGui.QColor(227, 255, 227), QtGui.QColor(255, 255, 255)
        for i, props in enumerate(self.ui.command_dict.values()):
            sub = props.get('subscriber')
            if sub:
                is_running = sub.get()
                self.ui.qlistwidget_commands.item(i).setBackground(green if is_running else white)

    def _update_fps_counter(self):
        """Updates the status bar with GUI and camera FPS."""
        if self.ui.counter % 80 == 0:
            current_time = time.time()
            time_delta = current_time - self.ui.previous_time
            if time_delta > 0:
                frames = self.ui.camera_manager.worker.frames if self.ui.camera_manager.worker else 0
                
                # Calculate NT Latency (Data Age)
                # This measures time in ms since the last timestamp update was received
                # Use match_time as the heartbeat since _timestamp is gone
                match_time_sub = self.ui.widget_dict['qlabel_matchtime'].get('subscriber')
                latency = -1.0
                if match_time_sub:
                    atomic_ts = match_time_sub.getAtomic()
                    if atomic_ts.time != 0:
                        # ntcore._now() and atomic_ts.time are both in microseconds. Divide by 1000 for ms.
                        latency = (ntcore._now() - atomic_ts.time) / 1000.0
                
                msg = f'GUI: {80/time_delta:.1f}Hz | CAM: {(frames - self.ui.previous_frames)/time_delta:.1f}Hz | LATENCY: {latency:.1f}ms'
                self.ui.statusBar().showMessage(msg)
            self.ui.previous_time = current_time
            self.ui.previous_frames = frames

    # --------------------------------------------------------------------------
    # Dispatcher Functions (Called from the self.updaters dictionary)
    # --------------------------------------------------------------------------

    def _update_indicator(self, props):
        """ Updates a label with a style based on state of the boolean tied to it """
        # TODO - maybe separate this based on whether it's monitoring a robot state boolean vs a command currently running
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return

        is_on = sub.get()
        
        # Optimization: If value hasn't changed, skip update.
        # EXCEPTION: If the widget is ON and has 'flash' enabled, we must update because self.flash_on toggles.
        if is_on == props.get('last_value') and not (props.get('flash') and is_on):
            return
        props['last_value'] = is_on

        if 'style_on' in props:  # allow a custom style for each indicator
            style = props['style_on'] if is_on else props['style_off']
        elif 'flash' in props and is_on:
            style = self.STYLE_FLASH_ON if self.flash_on else self.STYLE_FLASH_OFF
        else:
            style = self.STYLE_ON if is_on else self.STYLE_OFF  # go with the default
        widget.setStyleSheet(style)

    def _update_lcd(self, props):
        """ Updates a numeric LCD widget with an integer """
        # TODO - replace these with a better custom LCD-font widget
        sub, widget = props.get('subscriber'), props.get('widget')
        if sub and widget:
            value = int(sub.get()) # Convert to int for display and comparison
            if value == props.get('last_value'):
                return
            props['last_value'] = value
            widget.display(str(value))

    def _update_monitor(self, props):
        """ Updates a warninglabel with "good" and "bad" colors (set in UI's init) based on the value """
        sub, widget = props.get('subscriber'), props.get('widget')
        if sub and widget:
            value = sub.get()
            if value == props.get('last_value'):
                return
            props['last_value'] = value
            widget.set_value(value)

    def _update_combo(self, props):
        """ At the moment this is just for the autonomous routines combo box but it could be for any dropdown """
        sub, widget = props.get('subscriber'), props.get('widget')
        # print(f'found combo on {props.get("widget_name")} at {self.ui.counter} with sub {sub} and widget {widget}')
        # there is a problem with using shortcut checks for validity - the combobox is not True if it is empty (len 0)
        # so just check for None
        if sub is None or widget is None:
            return

        new_list = sub.get()
        list_changed = False
        if new_list != props.get('last_value'):
            props['last_value'] = new_list
            list_changed = True
            
            current_text = widget.currentText()
            widget.blockSignals(True)
            widget.clear()
            widget.addItems(new_list)
            if current_text in new_list:
                widget.setCurrentText(current_text)
            widget.blockSignals(False)
            self.ui.autonomous_list = new_list

        # SendableChooser publishes actual state to 'active', and reads requests from 'selected'
        active_sub = props.get('active_subscriber')
        selected_sub = props.get('selected_subscriber')
        
        robot_routine = active_sub.get() if active_sub else (selected_sub.get() if selected_sub else "")

        if robot_routine:
            if list_changed or robot_routine != widget.currentText():
                widget.blockSignals(True)
                widget.setCurrentText(robot_routine)
                widget.blockSignals(False)

    def _update_numeric_combo(self, props):
        """
        Updates a QComboBox from a numeric NetworkTables topic.
        This is a simplified version of _update_combo that only syncs a value, not a list of options.
        """
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return

        # NT sends a number, but the combo box holds strings.
        # We need to format the number to match an item in the combo box.
        nt_value = sub.get()
        nt_value_str = str(int(nt_value))  # Assuming integer values like '0', '1', '2'

        # Optimization: If the UI already shows the correct value, do nothing.
        if nt_value_str == widget.currentText():
            return

        # Block signals to prevent this programmatic change from firing a 'currentTextChanged' signal.
        widget.blockSignals(True)
        widget.setCurrentText(nt_value_str)
        widget.blockSignals(False)

    def _update_time(self, props):
        """ This is for the match time remaining widget - lame in sim but correct for matches """
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return

        match_time = sub.get()

        # In simulation (server index 1), match time counts up indefinitely. 
        # Convert it to a repeating 160-second countdown.
        if self.ui.nt_manager.server_index == 1:
            match_time = 160.0 - (match_time % 160.0)

        # If time is low, we flash, so we must update. If time is high, we only update if the integer second changes.
        val_int = int(match_time)
        if match_time >= 30 and val_int == props.get('last_value'):
            return
        props['last_value'] = val_int

        if match_time < 30:
            widget.setText(f'* {int(match_time)} *')
            widget.setStyleSheet(self.STYLE_FLASH_ON if self.flash_on else self.STYLE_FLASH_OFF)
        else:
            widget.setText(str(int(match_time)))
            widget.setStyleSheet(self.STYLE_HIGH)

    def _update_position(self, props):
        """  This was specific for CrankSinatra's shot position """
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return

        config = sub.get()
        if config == props.get('last_value'):
            return
        props['last_value'] = config
        # This logic seems to always result in STYLE_ON, may need review
        position_style = self.STYLE_ON if config.upper() not in ['LOW_SHOOT', 'INTAKE'] else self.STYLE_ON
        widget.setText(f'DIST: {config.upper()}')
        widget.setStyleSheet(position_style)

    def _update_active_hub(self, props):
        """ Updates the active hub indicator based on GameSpecificMessage and match time """
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return
            
        game_data = sub.get()  # 'R', 'B', or empty string
        
        # Grab the match time (assumes a continuous 160 -> 0 countdown)
        match_time_sub = self.ui.widget_dict.get('qlabel_matchtime', {}).get('subscriber')
        match_time = match_time_sub.get() if match_time_sub else 0
        
        # Convert simulation count-up to a 160-second countdown loop
        if self.ui.nt_manager.server_index == 1:
            match_time = 160.0 - (match_time % 160.0)

        active = "BOTH"
        countdown = 0
        
        # 2026 REBUILT FRC Timings: 160s total match time
        if match_time > 140: # Auto (20s)
            countdown = match_time - 140
        elif match_time > 130: # Transition Shift (10s)
            countdown = match_time - 130
        elif match_time > 30:
            # Teleop Shifts (four 25s periods)
            if match_time > 105:
                countdown = match_time - 105
                shift_num = 1
            elif match_time > 80:
                countdown = match_time - 80
                shift_num = 2
            elif match_time > 55:
                countdown = match_time - 55
                shift_num = 3
            else:
                countdown = match_time - 30
                shift_num = 4
                
            # 'R' means Red won auto, so Red's Hub is INACTIVE first (Blue is ACTIVE first).
            if game_data == "R":
                red_inactive_first = True
            elif game_data == "B":
                red_inactive_first = False
            else:
                red_inactive_first = None
                
            if red_inactive_first is not None:
                if shift_num % 2 != 0: # Shifts 1 and 3
                    active = "BLUE" if red_inactive_first else "RED"
                else: # Shifts 2 and 4
                    active = "RED" if red_inactive_first else "BLUE"
        else:
            # Endgame (30s)
            countdown = match_time
            
        current_state = f"{active}_{int(countdown)}"
        if current_state == props.get('last_state'):
            return
        props['last_state'] = current_state
        
        if active == "RED":
            style = "border: 7px; border-radius: 7px; background-color: rgb(225, 0, 0); color: rgb(255, 255, 255);"
            text = f"RED ACTIVE: {int(countdown)}s"
        elif active == "BLUE":
            style = "border: 7px; border-radius: 7px; background-color: rgb(0, 0, 225); color: rgb(255, 255, 255);"
            text = f"BLUE ACTIVE: {int(countdown)}s"
        else:
            # Gradient for Both (Half Red, Half Blue)
            style = (
                "border: 7px; border-radius: 7px; color: rgb(255, 255, 255); "
                "background: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
                "stop:0 rgb(225, 0, 0), stop:0.8 rgb(225, 0, 0), "
                "stop:0.8 rgb(0, 0, 225), stop:1 rgb(0, 0, 225));"
            )
            text = f"BOTH ACTIVE: {int(countdown)}s"

        widget.setStyleSheet(style)
        widget.setText(text)

    def _update_hub(self, props):
        """Legacy hub update logic from 2023. May need removal."""
        hub_targets_sub = self.ui.widget_dict['hub_targets'].get('subscriber')
        if not hub_targets_sub:
            return

        hub_targets = hub_targets_sub.get()
        if hub_targets > 0:
            hub_rotation = self.ui.widget_dict['hub_rotation']['subscriber'].get() - 5
            shooter_rpm = 2000
            shooter_distance = shooter_rpm * 0.00075
            center_offset = shooter_distance * -np.sin(hub_rotation * 3.14159 / 180)
            hub_x = 205 + center_offset * (380 / 1.2)
            self.ui.qlabel_ball.move(int(hub_x), 190)
            if self.ui.qlabel_ball.isHidden():
                self.ui.qlabel_ball.show()
        else:
            if not self.ui.qlabel_ball.isHidden():
                self.ui.qlabel_ball.hide()
