
import time
from datetime import datetime
from ntcore import NetworkTableInstance


class NTManager:
    def __init__(self, ui):
        self.ui = ui
        self.ntinst = NetworkTableInstance.getDefault()
        self.servers = ["10.24.29.2", "127.0.0.1"]
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.server_index = 0
        self.ntinst.setServerTeam(2429)

    def reconnect(self):
        sleep_time = 0.1
        self.ntinst.stopClient()
        time.sleep(sleep_time)
        self.ntinst.disconnect()
        time.sleep(sleep_time)
        self.ntinst = NetworkTableInstance.getDefault()
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        # Reconnect to whichever server we were on.  This used to always go back to the team server
        # while server_index stayed 1, so after "swap to sim" + reconnect the GUI was on the robot
        # but still doing the sim match-time math (ui_updater checks server_index == 1).
        if self.server_index == 0:
            self.ntinst.setServerTeam(2429)
        else:
            self.ntinst.setServer(server_name=self.servers[self.server_index])
        time.sleep(sleep_time)
        self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Reconnected to NetworkTables ({self.servers[self.server_index]}).')

    def increment_server(self):
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.servers)
        next_server = self.servers[self.server_index]
        self.ntinst.setServer(server_name=next_server)
        print(f'Changed server from {current_server} to {next_server}')
        self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Changed server from {current_server} to {next_server} ... wait 5s')

    def report_nt_status(self):
        if self.ntinst.isConnected():
            try:
                connection = self.ntinst.getConnections()[0]
                id, ip = connection.remote_id, connection.remote_ip
                self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: id={id}, ip={ip}')
            except IndexError:
                 self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: Connected, but no connection info.')
        else:
            self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: Disconnected')
