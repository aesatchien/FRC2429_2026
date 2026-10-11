
import time
import urllib.request
from datetime import datetime
import cv2
from PyQt6.QtCore import QObject, pyqtSignal, QThread, Qt
from PyQt6.QtGui import QImage, QPixmap
from config import DEFAULT_CAMERA

# THREADING RULE: Qt widgets and QPixmaps may only be touched on the GUI (main) thread.
# The worker below runs in its own QThread, so it never reads or writes a widget.  It reads the
# camera choice from plain attributes the GUI thread keeps up to date (CameraManager._on_*),
# builds a QImage (which IS safe off the GUI thread), and hands it over with a signal.  The
# GUI thread turns it into a pixmap.  Touching widgets from the worker was the old behavior
# and can crash or freeze the whole dashboard at random.


class CameraWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)
    frame_ready = pyqtSignal(QImage)  # delivered to the GUI thread
    status = pyqtSignal(str)          # lines for the status box

    def __init__(self, manager):
        super().__init__()
        self.manager = manager  # only its plain-Python selection attributes are read here
        self.running = False
        self.frames = 0
        self.previous_read_time = 0
        self.max_fps = 20
        self.error_count = 0
        self.cv_backend = cv2.CAP_FFMPEG

    def stop(self):
        self.running = False

    def _selected_url(self):
        camera_dict = self.manager.ui.camera_dict
        cam = DEFAULT_CAMERA if self.manager.autoswitch else self.manager.selected_camera
        # you can change the auto-switch logic here - e.g. when elevator is low or shooter is on, choose another camera
        props = camera_dict.get(cam)
        return props.get('URL') if props else None

    def run(self):
        self.running = True
        old_url = ''
        cap = None
        try:
            # Checking the streams can take ~1 s each when cameras are down, so it happens here, not on the GUI thread
            urls = {d['URL'] for d in self.manager.ui.camera_dict.values() if 'URL' in d}
            if not any(CameraManager.check_url(url) for url in urls):
                self.status.emit(f'{datetime.today().strftime("%H:%M:%S")}: No valid camera servers, unable to start camera')
                return
            self.status.emit(f'{datetime.today().strftime("%H:%M:%S")}: Camera thread running')

            while self.running:
                url = self._selected_url()
                if url is None:
                    time.sleep(0.1)
                    continue

                if url != old_url:
                    if cap is not None:
                        cap.release()

                    retries = 0
                    max_retries = 10
                    while retries < max_retries and self.running:
                        cap = cv2.VideoCapture(url, self.cv_backend)
                        if cap.isOpened():
                            print(f"Successfully opened stream: {url}")
                            break
                        else:
                            retries += 1
                            print(f"Cannot open stream: {url}. Retrying ({retries}/{max_retries})...")
                            time.sleep(0.5)

                    if not cap or not cap.isOpened():
                        self.status.emit(f'{datetime.today().strftime("%H:%M:%S")}: Could not open {url} after {max_retries} tries - camera thread stopped')
                        return # Exit the thread

                    old_url = url

                now = time.time()
                if now - self.previous_read_time > 1.0 / self.max_fps:
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret:
                            self.frames += 1
                            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            h, w, ch = rgb.shape
                            # .copy() so the QImage owns its pixels - rgb's buffer is freed when this loop moves on
                            self.frame_ready.emit(QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy())
                            self.previous_read_time = now
                        else:
                            print("[ERROR] OpenCV stopped receiving frames. Retrying...")
                            cap.release()
                            old_url = ''  # force retry loop to re-run on next iteration
                    else:
                        time.sleep(0.5) # Wait if cap is not ready
                else:
                    if cap.isOpened():
                        cap.grab()  # Flush buffer to prevent lag buildup
                    time.sleep(0.001)  # Yield CPU between frame-rate checks; avoids busy-spin

        except Exception as e:
            print(f'{datetime.today().strftime("%H%M%S")} Camera worker crashed: {e}')
        finally:
            if cap is not None:
                cap.release()
            self.finished.emit()

class CameraManager:
    def __init__(self, ui):
        self.ui = ui
        self.thread = None
        self.worker = None
        # the worker's view of the camera controls, kept current by the GUI thread
        self.autoswitch = ui.qradiobutton_autoswitch.isChecked()
        self.selected_camera = ui.qcombobox_cameras.currentText()
        ui.qradiobutton_autoswitch.toggled.connect(self._on_autoswitch)
        ui.qcombobox_cameras.currentTextChanged.connect(self._on_camera_selected)

    def _on_autoswitch(self, checked):
        self.autoswitch = checked

    def _on_camera_selected(self, text):
        self.selected_camera = text

    def show_frame(self, image):
        """GUI thread: scale the worker's frame to the label and show it."""
        label = self.ui.qlabel_camera_view
        scaled = image.scaled(label.width(), label.height(), Qt.AspectRatioMode.KeepAspectRatio)
        label.setPixmap(QPixmap.fromImage(scaled))

    @staticmethod
    def check_url(url):
        try:
            # Explicitly close the stream so we don't exhaust the camera server's client limit
            with urllib.request.urlopen(url, timeout=1.0) as response:
                return response.getcode() == 200
        except Exception as e:
            print(f'Failure: attempted to check {url} with exception {e}')
        return False

    def toggle_camera_thread(self):
        # Act as a true toggle: If it is already running, shut it down.
        if self.thread is not None and self.thread.isRunning():
            self.worker.stop()
            self.thread.quit()
            self.ui.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Terminating camera thread (User toggled off)')
            return

        restarting = self.thread is not None
        self.autoswitch = self.ui.qradiobutton_autoswitch.isChecked()
        self.selected_camera = self.ui.qcombobox_cameras.currentText()
        self.thread = QThread()
        self.worker = CameraWorker(manager=self)
        self.worker.moveToThread(self.thread)
        self.worker.frame_ready.connect(self.show_frame)
        self.worker.status.connect(self.ui.qt_text_status.appendPlainText)
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.thread.start()
        self.ui.qt_text_status.appendPlainText(
            f'{datetime.today().strftime("%H:%M:%S")}: {"Restarting" if restarting else "Starting"} camera thread - checking camera servers')
