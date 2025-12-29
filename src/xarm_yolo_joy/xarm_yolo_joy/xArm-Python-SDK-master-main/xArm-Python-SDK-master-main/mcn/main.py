
from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import QTimer
from gui import Ui_MainWindow

from anhooa import (
    cmd_clear_error, cmd_enable_all_joints, send_command, parse_response,
    read_cartesian_position, move_xyz_offset_gui, set_speed_percent,
    return_to_zero_from_speed,
    set_robot_target, get_last_io_hex,
    open_gripper_safe, close_gripper_safe, stop_gripper_safe
)

import sys

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Key state cho chuyển động liên tục
        self.key_state = {"xp":0, "xm":0, "yp":0, "ym":0, "zp":0, "zm":0}

        # ===== KẾT NỐI CÁC NÚT (nếu có trong GUI) =====
        # Connect IP
        if hasattr(self.ui, "pushButton"):
            self.ui.pushButton.clicked.connect(self.handle_connect)

        # Enable
        if hasattr(self.ui, "pushButton_2"):
            self.ui.pushButton_2.clicked.connect(self.handle_enable_robot)

        # Clear Error
        if hasattr(self.ui, "pushButton_3"):
            self.ui.pushButton_3.clicked.connect(self.handle_clear_error)

        # Zero from Speed
        if hasattr(self.ui, "pushButton_12"):
            self.ui.pushButton_12.clicked.connect(self.on_zero_position)

        # Gripper (giả định 3 nút Open/Close/Stop)
        if hasattr(self.ui, "pushButton_8"):
            self.ui.pushButton_8.clicked.connect(self.handle_gripper_open)
        if hasattr(self.ui, "pushButton_9"):
            self.ui.pushButton_9.clicked.connect(self.handle_gripper_close)
        if hasattr(self.ui, "pushButton_10"):
            self.ui.pushButton_10.clicked.connect(self.handle_gripper_stop)

        # Speed slider + label
        if hasattr(self.ui, "horizontalSlider_7"):
            self.ui.horizontalSlider_7.valueChanged.connect(self.update_speed_label)

        # Speed - / +
        if hasattr(self.ui, "pushButton_14"):
            self.ui.pushButton_14.clicked.connect(lambda: self.bump_speed(-5))
        if hasattr(self.ui, "pushButton_15"):
            self.ui.pushButton_15.clicked.connect(lambda: self.bump_speed(+5))

        # Timer chuyển động liên tục (nếu có phím giữ)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.continuous_move)
        self.timer.start(60)  # ~16Hz

        # Timer đọc Pose định kỳ để hiển thị
        self.pose_timer = QTimer(self)
        self.pose_timer.timeout.connect(self.update_pose_view)
        self.pose_timer.start(500)

    # ======= Hiển thị gói tin hex =======
    def update_req_resp_hex(self):
        if hasattr(self.ui, "label_15") and hasattr(self.ui, "label_16"):
            req_hex, resp_hex = get_last_io_hex()
            if req_hex:
                self.ui.label_15.setText(req_hex)
            if resp_hex:
                self.ui.label_16.setText(resp_hex)

    # ======= Handlers =======
    def handle_connect(self):
        ip = ""
        if hasattr(self.ui, "textEdit"):
            ip = self.ui.textEdit.toPlainText().strip()
        if not ip:
            if hasattr(self.ui, "label_16"):
                self.ui.label_16.setText("⚠️ Chưa nhập IP")
            return
        msg = set_robot_target(ip, 502)
        if hasattr(self.ui, "label_16"):
            self.ui.label_16.setText(msg)

    def handle_enable_robot(self):
        try:
            resp = send_command(cmd_enable_all_joints())
            result = parse_response(resp)
            if hasattr(self.ui, "label_16"):
                self.ui.label_16.setText(result)
            self.update_req_resp_hex()
        except Exception as e:
            if hasattr(self.ui, "label_16"):
                self.ui.label_16.setText(f"Lỗi: {e}")

    def handle_clear_error(self):
        try:
            resp = send_command(cmd_clear_error())
            result = parse_response(resp)
            if hasattr(self.ui, "label_16"):
                self.ui.label_16.setText(result)
            self.update_req_resp_hex()
        except Exception as e:
            if hasattr(self.ui, "label_16"):
                self.ui.label_16.setText(f"Lỗi: {e}")

    def handle_gripper_open(self):
        if hasattr(self.ui, "label_16"):
            self.ui.label_16.setText(str(open_gripper_safe()))
        self.update_req_resp_hex()

    def handle_gripper_close(self):
        if hasattr(self.ui, "label_16"):
            self.ui.label_16.setText(str(close_gripper_safe()))
        self.update_req_resp_hex()

    def handle_gripper_stop(self):
        if hasattr(self.ui, "label_16"):
            self.ui.label_16.setText(str(stop_gripper_safe()))
        self.update_req_resp_hex()

    def on_zero_position(self):
        if hasattr(self.ui, "label_16"):
            self.ui.label_16.setText(str(return_to_zero_from_speed()))
        self.update_req_resp_hex()

    def bump_speed(self, delta: int):
        if hasattr(self.ui, "horizontalSlider_7"):
            v = max(0, min(100, self.ui.horizontalSlider_7.value() + delta))
            self.ui.horizontalSlider_7.setValue(v)

    def update_speed_label(self, val: int):
        result = set_speed_percent(val)
        if hasattr(self.ui, "label_14"):
            self.ui.label_14.setText(f"{val}%")
        # không ghi đè label_16 để chừa hiển thị response

    def continuous_move(self):
        # Nếu bạn có phím giữ trong GUI, cập nhật self.key_state theo sự kiện phím
        # Ở đây demo: không dùng phím, hàm sẽ không gửi move nếu không có dx/dy/dz
        return

    def update_pose_view(self):
        pose = read_cartesian_position()
        if isinstance(pose, tuple) and len(pose) == 6:
            x, y, z, rx, ry, rz = pose
            if hasattr(self.ui, "label_12"):
                self.ui.label_12.setText(f"{x:.1f}")
            if hasattr(self.ui, "label_17"):
                self.ui.label_17.setText(f"{y:.1f}")
            if hasattr(self.ui, "label_18"):
                self.ui.label_18.setText(f"{z:.1f}")
            if hasattr(self.ui, "label_19"):
                self.ui.label_19.setText(f"{rx:.2f}")
            if hasattr(self.ui, "label_20"):
                self.ui.label_20.setText(f"{ry:.2f}")
            if hasattr(self.ui, "label_21"):
                self.ui.label_21.setText(f"{rz:.2f}")
            self.update_req_resp_hex()
        else:
            # có thể hiển thị chuỗi lỗi nếu muốn
            pass

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
