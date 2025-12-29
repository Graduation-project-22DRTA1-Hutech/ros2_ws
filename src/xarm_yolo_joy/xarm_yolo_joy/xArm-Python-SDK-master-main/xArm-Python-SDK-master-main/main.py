from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import QTimer, QStringListModel
import sys
from gui import Ui_MainWindow  # giữ nguyên .ui đã compile
from PySide2.QtCore import Qt
from PySide2.QtGui import QFont
import math



# anhooa API
from anhooa import (
    open_gripper, close_gripper, stop_gripper,
    move_xyz_offset_gui, set_speed_percent,
    CURR_X, CURR_Y, CURR_Z, TCP_SPEED_PERCENT,
    cmd_clear_error, send_command, parse_response,
    cmd_enable_all_joints, read_cartesian_position,
    return_to_zero_from_speed, parse_response_status)

from sdf import (cmd_move_xyz_speed_based, send_cmd,parse_resp
    
)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        

        
        # Căn giữa & font cố định cho 2 khung
        self.ui.label_15.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.ui.label_16.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        
        mono = QFont("Consolas")                 # hoặc "Courier New"
        mono.setStyleHint(QFont.Monospace)
        mono.setFixedPitch(True)
        mono.setPixelSize(16)                    # <— giữ 1 size (đổi 15/16 nếu muốn to hơn)

        self.ui.label_15.setFont(mono)
        self.ui.label_16.setFont(mono)
        self.ui.listView.setFont(mono)

        # (tùy chọn) cho phép bôi đen copy, và xuống dòng nếu quá dài
        self.ui.label_15.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.ui.label_16.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.ui.label_15.setWordWrap(True)
        self.ui.label_16.setWordWrap(True)


        # ==== Timer cập nhật vị trí TCP (5 Hz) ====
        self.timer_pos = QTimer(self)
        self.timer_pos.timeout.connect(self.update_position)
        self.timer_pos.start(200)

        # ==== List log flags ====
        self.flag_model = QStringListModel([])
        self.ui.listView.setModel(self.flag_model)


        # ==== Gripper / Robot buttons ====
        self.ui.pushButton_12.clicked.connect(self.handle_gripper_open)   # Open
        self.ui.pushButton_13.clicked.connect(self.handle_gripper_close)  # Close
        self.ui.pushButton_16.clicked.connect(self.handle_gripper_stop)   # Stop
        self.ui.pushButton_9.clicked.connect(self.handle_enable_robot)
        self.ui.pushButton_8.clicked.connect(self.handle_clear_error)
        self.ui.pushButton_10.clicked.connect(self.on_zero_position)
        self.ui.pushButton_17.clicked.connect(self.handle_move)

        self.speed = 50  # giá trị tốc độ mặc định
        self.x = 87
        self.y = 0
        self.z = 154.2

        # ==== Key states ====
        self.key_state = {"xp": False, "xm": False,
                          "yp": False, "ym": False,
                          "zp": False, "zm": False}

        # press / release
        self.ui.pushButton_2.pressed.connect(lambda: self.set_key("xp", True))
        self.ui.pushButton_2.released.connect(lambda: self.set_key("xp", False))
        self.ui.pushButton_3.pressed.connect(lambda: self.set_key("xm", True))
        self.ui.pushButton_3.released.connect(lambda: self.set_key("xm", False))
        self.ui.pushButton_4.pressed.connect(lambda: self.set_key("yp", True))
        self.ui.pushButton_4.released.connect(lambda: self.set_key("yp", False))
        self.ui.pushButton_5.pressed.connect(lambda: self.set_key("ym", True))
        self.ui.pushButton_5.released.connect(lambda: self.set_key("ym", False))
        self.ui.pushButton_6.pressed.connect(lambda: self.set_key("zp", True))
        self.ui.pushButton_6.released.connect(lambda: self.set_key("zp", False))
        self.ui.pushButton_7.pressed.connect(lambda: self.set_key("zm", True))
        self.ui.pushButton_7.released.connect(lambda: self.set_key("zm", False))

        # ==== Timer move liên tục (20 Hz) ====
        self.move_timer = QTimer(self)
        self.move_timer.timeout.connect(self.continuous_move)
        self.move_timer.start(100)

        # ==== Slider tốc độ ====
        self.ui.horizontalSlider_7.setMinimum(0)
        self.ui.horizontalSlider_7.setMaximum(100)
        self.ui.horizontalSlider_7.setValue(TCP_SPEED_PERCENT)
        self.ui.horizontalSlider_7.valueChanged.connect(self.update_speed_label)
    
    def set_key(self, key, value):
        self.key_state[key] = value

    def _hex0x(self, b: bytes) -> str:
        return " ".join(f"0x{x:02X}" for x in b)
    # ===== Format helpers =====
    def _hex_list(self, data: bytes) -> str:
        return " ".join(f"0x{x:02X}" for x in (data or b""))

    def _show_req_resp_block(self, req_b: bytes = None, resp_b: bytes = None):
        # ----- Request -----
        if isinstance(req_b, (bytes, bytearray)) and len(req_b) >= 7:
            # header
            tid_bytes    = req_b[0:2]
            proto_bytes  = req_b[2:4]
            len_bytes    = req_b[4:6]
            reg_byte     = req_b[6:7]
            params_bytes = req_b[7:]
            req_text = (
                "\n"
                f"TID: {self._hex_list(tid_bytes)}\n"
                f"Protocal: {self._hex_list(proto_bytes)}\n"
                f"Length: {self._hex_list(len_bytes)}\n"
                f"Register: {self._hex_list(reg_byte)}\n"
                f"Param: {self._hex_list(params_bytes) if params_bytes else '-'}"
            )
        else:
            req_text = "Request:\nTID: -\nProtocal: -\nLength: -\nRegister: -\nParam: -"

        # ----- Response -----
        if isinstance(resp_b, (bytes, bytearray)) and len(resp_b) >= 8:
            # length trong header = số byte body (reg + status bytes)
            body_len = (resp_b[4] << 8) | resp_b[5]
            # reg 1 byte, phần còn lại là status/payload
            reg_byte = resp_b[6:7]
            status_bytes = resp_b[7:8]
            params_bytes = resp_b[8:]  # Các tham số (param) sẽ bắt đầu từ byte 8 trở đi

            resp_text = (
                "\n"
                f"Register: {self._hex_list(reg_byte)}\n"
                f"Status: {self._hex_list(status_bytes) if status_bytes else '-'}\n"
                f"Param: {self._hex_list(params_bytes) if params_bytes else '-'}"  # Hiển thị tham số
            )
        else:
            resp_text = "Response:\nRegister: -\nStatus: -\nParam: -"

        # đổ ra 2 label
        self.ui.label_15.setText(req_text)
        self.ui.label_16.setText(resp_text)



    def _call_req_resp(self, func, *args, **kwargs):
        """
        Gọi API anhooa:
        - Nếu hàm hỗ trợ return_req=True -> trả (req, resp)
        - Nếu không -> trả resp
        """
        try:
            ret = func(*args, return_req=True, **kwargs)
        except TypeError:
            ret = func(*args, **kwargs)

        req, resp = None, None
        if isinstance(ret, tuple) and len(ret) == 2:
            req, resp = ret
        elif isinstance(ret, dict) and "req" in ret and "resp" in ret:
            req, resp = ret["req"], ret["resp"]
        elif isinstance(ret, (bytes, bytearray)):
            resp = ret
        else:
            resp = ret
        return req, resp

    def _show_req_resp_labels(self, req: bytes = None, resp: bytes = None):
        # Label 15: Req
        if isinstance(req, (bytes, bytearray)) and len(req) >= 7:
            try:
                tid   = int.from_bytes(req[0:2], 'big')
                proto = int.from_bytes(req[2:4], 'big')
                leng  = int.from_bytes(req[4:6], 'big')
                reg   = req[6]
                params = req[7:]
                req_str = (f"Req: TID({tid}) Proto({proto}) Len({leng}) "
                           f"Reg(0x{reg:02X}) Params({self._hex0x(params) if params else '-'})")
            except Exception as e:
                req_str = f"Req parse error: {e}"
        elif req is None:
            req_str = "Req: -"
        else:
            req_str = "Req: (không hợp lệ)"
        self.ui.label_15.setText(req_str)

        # Label 16: Resp
        if isinstance(resp, (bytes, bytearray)) and len(resp) >= 8:
            rreg  = resp[6]
            state = resp[7]
            resp_str = f"Reg(0x{rreg:02X})| State(0x{state:02X})"
        elif resp is None:
            resp_str = "Resp: -"
        else:
            resp_str = "Resp: (trống/không hợp lệ)"
        self.ui.label_16.setText(resp_str)

    def _append_flags(self, resp: bytes, prefix: str = ""):
        flags = parse_response_status(resp)        # list[str]
        line = ("{}: ".format(prefix) if prefix else "") + " ; ".join(flags)
        logs = self.flag_model.stringList()
        logs.append(line)
        self.flag_model.setStringList(logs)
        self.ui.listView.scrollToBottom()

    # ---------- Live pose ----------
    def update_position(self):
        try:
            res = read_cartesian_position()
            if isinstance(res, tuple) and len(res) == 6:
                x, y, z, rx, ry, rz = res
                self.ui.textEdit_2.setPlainText(f"{x:.1f}")
                self.ui.textEdit_3.setPlainText(f"{y:.1f}")
                self.ui.textEdit_4.setPlainText(f"{z:.1f}")
                self.ui.textEdit_5.setPlainText(f"{rx:.2f}")
                self.ui.textEdit_7.setPlainText(f"{ry:.2f}")
                self.ui.textEdit_6.setPlainText(f"{rz:.2f}")
            else:
                self.ui.label_16.setText(str(res))
        except Exception as e:
            self.ui.label_16.setText(f"Lỗi pose: {e}")

    # ---------- Command handlers ----------
    def handle_enable_robot(self):
        try:
            req_b, resp_b = self._call_req_resp(send_command, cmd_enable_all_joints())
            self._show_req_resp_block(req_b, resp_b)
            if resp_b: self._append_flags(resp_b, "Enable")
        except Exception as e:
            self.ui.label_16.setText(f"Lỗi: {e}")


    def handle_clear_error(self):
        try:
            req_b, resp_b = self._call_req_resp(send_command, cmd_clear_error())
            self._show_req_resp_block(req_b, resp_b)
            if resp_b: self._append_flags(resp_b, "ClearError")
        except Exception as e:
            self.ui.label_16.setText(f"Lỗi: {e}")

    def handle_gripper_open(self):
        try:
            req_b, resp_b = self._call_req_resp(open_gripper)
            self._show_req_resp_block(req_b, resp_b)          # dùng formatter block
            if resp_b:
                self._append_flags(resp_b, "Gripper OPEN")
        except Exception as e:
            self.ui.label_16.setText(f"Lỗi: {e}")

    def handle_gripper_close(self):
        try:
            req_b, resp_b = self._call_req_resp(close_gripper)
            self._show_req_resp_block(req_b, resp_b)          # dùng formatter block
            if resp_b:
                self._append_flags(resp_b, "Gripper CLOSE")
        except Exception as e:
            self.ui.label_16.setText(f"Lỗi: {e}")

    def handle_gripper_stop(self):
        try:
            # cố gắng lấy cả req/resp cho từng kênh
            try:
                results = stop_gripper(return_req=True)   # {ch: (req, resp)}
            except TypeError:
                results = stop_gripper()                  # fallback

            if isinstance(results, dict):
                for ch, item in results.items():
                    if isinstance(item, tuple) and len(item) == 2:
                        req_b, resp_b = item
                    elif isinstance(item, dict):
                        req_b, resp_b = item.get("req"), item.get("resp")
                    else:
                        req_b, resp_b = None, item

                    # HIỂN THỊ THEO FORMAT BLOCK (Request/Response)
                    self._show_req_resp_block(req_b, resp_b)

                    # Log trạng thái cho từng kênh
                    if resp_b:
                        self._append_flags(resp_b, f"Gripper STOP ch{ch}")
            else:
                # trường hợp API không trả dict per-channel
                req_b, resp_b = (results.get("req"), results.get("resp")) if isinstance(results, dict) else (None, results)
                self._show_req_resp_block(req_b, resp_b)
                if resp_b:
                    self._append_flags(resp_b, "Gripper STOP")

        except Exception as e:
            self.ui.label_16.setText(f"Lỗi: {e}")


    def on_zero_position(self):
        try:
            req_b, resp_b = self._call_req_resp(return_to_zero_from_speed)
            self._show_req_resp_block(req_b, resp_b)   # <- dùng block formatter
            if resp_b:
                self._append_flags(resp_b, "Zero Position")
        except Exception as e:
            self.ui.label_16.setText(f"Lỗi: {e}")

    # ---------- Move ----------
    def continuous_move(self):
        speed_scale = max(0, min(100, self.ui.horizontalSlider_7.value())) / 100.0
        step = 1.0 * speed_scale
        dx = (self.key_state["xp"] - self.key_state["xm"]) * step
        dy = (self.key_state["yp"] - self.key_state["ym"]) * step
        dz = (self.key_state["zp"] - self.key_state["zm"]) * step
        if dx or dy or dz:
            try:
                pose = read_cartesian_position()
                if not (isinstance(pose, tuple) and len(pose) >= 3):
                    self.ui.label_16.setText("Pose không hợp lệ để tính đích")
                    return
                cx, cy, cz = pose[0], pose[1], pose[2]
                tx, ty, tz = cx + dx, cy + dy, cz + dz

                req_b, resp_b = self._call_req_resp(move_xyz_offset_gui, dx=dx, dy=dy, dz=dz)

                # hiển thị đúng format
                self._show_req_resp_block(req_b, resp_b)

                if isinstance(resp_b, (bytes, bytearray)) and len(resp_b) >= 8:
                    self._append_flags(resp_b, f"Move tới X={tx:.2f} Y={ty:.2f} Z={tz:.2f}")
            except Exception as e:
                self.ui.label_16.setText(f"Lỗi move: {e}")

    # đơn bước (nếu cần)
    def move_and_update(self, dx=0, dy=0, dz=0):
        try:
            pose = read_cartesian_position()
            if not (isinstance(pose, tuple) and len(pose) >= 3):
                self.ui.label_16.setText("Pose không hợp lệ để tính đích")
                return
            cx, cy, cz = pose[0], pose[1], pose[2]
            tx, ty, tz = cx + dx, cy + dy, cz + dz

            req, resp = self._call_req_resp(move_xyz_offset_gui, dx=dx, dy=dy, dz=dz)
            self._show_req_resp_labels(req, resp)

            if isinstance(resp, (bytes, bytearray)) and len(resp) >= 8:
                self._append_flags(resp, f"Move tới X={tx:.2f} Y={ty:.2f} Z={tz:.2f}")
            else:
                self.ui.label_16.setText("Move: phản hồi trống/không hợp lệ")
        except Exception as e:
            self.ui.label_16.setText(f"Lỗi move: {e}")
    
    # ---------- Speed ----------
    def update_speed_label(self, val):
        result = set_speed_percent(val)  # chuỗi log trả về từ anhooa
        self.ui.label_14.setText(f"Tốc độ: {val}%")
        logs = self.flag_model.stringList()
        logs.append(result)
        self.flag_model.setStringList(logs)
        self.ui.listView.scrollToBottom()
    def handle_move(self):
        try:
            # Lấy tọa độ từ các trường nhập liệu trong GUI
            target_x = float(self.ui.textEdit_8.toPlainText())
            target_y = float(self.ui.textEdit_9.toPlainText())
            target_z = float(self.ui.textEdit_10.toPlainText())

            # Lấy tốc độ từ textEdit_11
            speed_mm_s = float(self.ui.textEdit_11.toPlainText())  # Lấy tốc độ từ textEdit_11

            # Tạo gói lệnh move
            pkt = cmd_move_xyz_speed_based(
                x_mm=target_x, y_mm=target_y, z_mm=target_z,
                rx_rad=math.pi, ry_rad=0, rz_rad=0,
                speed_mm_s=speed_mm_s, acc_mm_s2=50000,
                coord_system=0, absolute_pose=0, tid=71
            )

            # Gửi lệnh và nhận phản hồi
            req_b, resp_b = self._call_req_resp(send_cmd, pkt, 5.0)

            # In thông tin request và response lên giao diện
            self._show_req_resp_block(req_b, resp_b)

            # Ghi lại các flag phản hồi cho lệnh di chuyển
            if isinstance(resp_b, (bytes, bytearray)) and len(resp_b) >= 8:
                self._append_flags(resp_b, f"Move to X={target_x:.2f} Y={target_y:.2f} Z={target_z:.2f}")



        except Exception as e:
            self.ui.label_16.setText(f"Error: {e}")







if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
