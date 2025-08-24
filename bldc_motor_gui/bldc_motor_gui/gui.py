#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, csv, time, math, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from std_msgs.msg import Float32MultiArray

RPM_PER_RADPS = 60.0 / (2.0 * math.pi)

def kt_from_kv(kv_rpm_per_v: float) -> float:
    """Kt [Nm/A] ≈ 60/(2π*Kv[rpm/V]) = 9.5493 / Kv"""
    if kv_rpm_per_v <= 0:
        return 0.0
    return 9.5492965855 / kv_rpm_per_v

class MotorPlotNode(Node):
    def __init__(self):
        super().__init__('bldc_motor_plotter')

        # ---------- ROS Subscription ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        self.topic = '/bldc_motor/state'
        self.sub = self.create_subscription(
            Float32MultiArray, self.topic, self.listener_callback, qos
        )

        # Mesaj alan indeksleri (ayar panelinden değiştirilebilir)
        self.idx_omega   = 0   # rad/s
        self.idx_current = 1   # A (faz veya DC; elektrik güç hesabında DC varsayıyoruz)
        self.idx_vbus    = 2   # V (komut yerine gerçek DC varsa daha doğru)
        self.idx_torque  = 3   # Nm

        # ---------- Data Buffers ----------
        self.max_points = 2000
        self.torque_data = collections.deque(maxlen=self.max_points)
        self.speed_data  = collections.deque(maxlen=self.max_points)   # rad/s
        self.current_data= collections.deque(maxlen=self.max_points)
        self.vbus_data   = collections.deque(maxlen=self.max_points)
        self.power_data  = collections.deque(maxlen=self.max_points)
        self.eff_data    = collections.deque(maxlen=self.max_points)
        self.ts_data     = collections.deque(maxlen=self.max_points)   # timestamps

        self.last_rx_times = collections.deque(maxlen=100)             # msg rate hesap

        # ---------- Defaults (AK10-9 V2 KV100 datasheet) ----------
        # Kaynak: CubeMars ürün sayfasındaki tablo. (9:1, 24/48V; 15Nm nominal, 38Nm peak, 205/421 rpm nominal; 266/533 rpm no-load; 16.2/41.2A)
        self.default_ratio = 9.0
        self.kv = 100.0               # rpm/V
        self.kt_lock = True           # Kt'yi Kv'den türet
        self.kt = kt_from_kv(self.kv) # Nm/A (motor tarafı)
        self.r_phase = 0.0655         # Ω (65.5 mΩ, kaynaklarda değişebilir)
        self.l_phase = 60e-6          # H (yaklaşık)
        self.gear_eff = 0.9           # tahmini dişli verimi
        self.fixed_vbus_enabled = False
        self.fixed_vbus = 48.0

        # Limitler / rehber çizgiler
        self.torque_rated = 15.0      # Nm @ output
        self.torque_peak  = 38.0      # Nm @ output
        self.speed_rated_rpm_48 = 421 # rpm @ 48 V (nominal)
        self.speed_noload_rpm_48 = 533

        # Filtre / görselleştirme
        self.ma_window = 10           # hareketli ortalama pencere
        self.plot_paused = False
        self.recording = False
        self.csv_writer = None
        self.csv_fh = None

        # ---------- Qt UI ----------
        self.app = QtWidgets.QApplication(sys.argv)
        self.app.setApplicationName("BLDC Motor Live Monitor (AK10-9)")
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle("BLDC Motor Live Plots – AK10-9 Monitor")
        self.central = QtWidgets.QWidget()
        self.win.setCentralWidget(self.central)

        main_layout = QtWidgets.QHBoxLayout(self.central)

        # Left: Tabs with plots
        self.tabs = QtWidgets.QTabWidget()
        main_layout.addWidget(self.tabs, stretch=4)

        # Tab 1: Scatter/Maps (Torque on X)
        self.tab_scatter = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_scatter, "Haritalar (X: Tork)")
        scatter_layout = QtWidgets.QVBoxLayout(self.tab_scatter)
        self.gw_scatter = pg.GraphicsLayoutWidget(show=False)
        scatter_layout.addWidget(self.gw_scatter)

        # Speed vs Torque
        self.plot_speed = self.gw_scatter.addPlot(title="Speed (rpm) vs Torque (Nm)")
        self.plot_speed.setLabel('bottom', 'Torque', units='Nm')
        self.plot_speed.setLabel('left', 'Speed', units='rpm')
        self.curve_speed = self.plot_speed.plot(pen='y')
        # Limit çizgileri
        self.vline_t_rated_1 = pg.InfiniteLine(angle=90, pos=self.torque_rated, pen=pg.mkPen((200,200,200), style=QtCore.Qt.DashLine))
        self.vline_t_peak_1  = pg.InfiniteLine(angle=90, pos=self.torque_peak,  pen=pg.mkPen((255,150,150), style=QtCore.Qt.DashLine))
        self.plot_speed.addItem(self.vline_t_rated_1); self.plot_speed.addItem(self.vline_t_peak_1)

        self.gw_scatter.nextRow()

        # Power vs Torque
        self.plot_power = self.gw_scatter.addPlot(title="Mechanical Power (W) vs Torque (Nm)")
        self.plot_power.setLabel('bottom', 'Torque', units='Nm')
        self.plot_power.setLabel('left', 'Power', units='W')
        self.curve_power = self.plot_power.plot(pen='r')
        self.vline_t_rated_2 = pg.InfiniteLine(angle=90, pos=self.torque_rated, pen=pg.mkPen((200,200,200), style=QtCore.Qt.DashLine))
        self.vline_t_peak_2  = pg.InfiniteLine(angle=90, pos=self.torque_peak,  pen=pg.mkPen((255,150,150), style=QtCore.Qt.DashLine))
        self.plot_power.addItem(self.vline_t_rated_2); self.plot_power.addItem(self.vline_t_peak_2)

        self.gw_scatter.nextRow()

        # Efficiency vs Torque
        self.plot_eff = self.gw_scatter.addPlot(title="Efficiency (%) vs Torque (Nm)")
        self.plot_eff.setLabel('bottom', 'Torque', units='Nm')
        self.plot_eff.setLabel('left', 'Efficiency', units='%')
        self.curve_eff = self.plot_eff.plot(pen='g')
        self.vline_t_rated_3 = pg.InfiniteLine(angle=90, pos=self.torque_rated, pen=pg.mkPen((200,200,200), style=QtCore.Qt.DashLine))
        self.vline_t_peak_3  = pg.InfiniteLine(angle=90, pos=self.torque_peak,  pen=pg.mkPen((255,150,150), style=QtCore.Qt.DashLine))
        self.plot_eff.addItem(self.vline_t_rated_3); self.plot_eff.addItem(self.vline_t_peak_3)

        # Tab 2: Time Series
        self.tab_time = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_time, "Zaman Serisi")
        time_layout = QtWidgets.QVBoxLayout(self.tab_time)
        self.gw_time = pg.GraphicsLayoutWidget(show=False)
        time_layout.addWidget(self.gw_time)

        self.ts_plot_speed = self.gw_time.addPlot(title="Speed (rpm) vs Time")
        self.ts_plot_speed.setLabel('left', 'Speed', units='rpm')
        self.ts_plot_speed.setLabel('bottom', 'Time', units='s')
        self.ts_curve_speed = self.ts_plot_speed.plot(pen='y')

        self.gw_time.nextRow()
        self.ts_plot_torque = self.gw_time.addPlot(title="Torque (Nm) vs Time")
        self.ts_plot_torque.setLabel('left', 'Torque', units='Nm')
        self.ts_plot_torque.setLabel('bottom', 'Time', units='s')
        self.ts_curve_torque = self.ts_plot_torque.plot(pen='c')

        self.gw_time.nextRow()
        self.ts_plot_current = self.gw_time.addPlot(title="Current (A) vs Time")
        self.ts_plot_current.setLabel('left', 'Current', units='A')
        self.ts_plot_current.setLabel('bottom', 'Time', units='s')
        self.ts_curve_current = self.ts_plot_current.plot(pen='m')

        self.gw_time.nextRow()
        self.ts_plot_vbus = self.gw_time.addPlot(title="Bus Voltage (V) vs Time")
        self.ts_plot_vbus.setLabel('left', 'Voltage', units='V')
        self.ts_plot_vbus.setLabel('bottom', 'Time', units='s')
        self.ts_curve_vbus = self.ts_plot_vbus.plot(pen='w')

        # Right: Settings / status panel
        self.panel = self._build_settings_panel()
        main_layout.addWidget(self.panel, stretch=0)

        # ---------- Timers ----------
        # UI refresh
        self.ui_timer = QtCore.QTimer()
        self.ui_timer.timeout.connect(self.update_plots)
        self.ui_timer.start(100)  # 10 Hz

        # rclpy spin entegrasyonu (Qt loop ile)
        self.ros_timer = QtCore.QTimer()
        self.ros_timer.timeout.connect(self._spin_ros_once)
        self.ros_timer.start(5)   # ~200 Hz non-blocking

        self.win.resize(1300, 850)
        self.win.show()

    # ---------- UI: Settings Panel ----------
    def _build_settings_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(panel)

        # Topic & indices
        grp_topic = QtWidgets.QGroupBox("Abonelik")
        f1 = QtWidgets.QFormLayout(grp_topic)
        self.le_topic = QtWidgets.QLineEdit(self.topic)
        self.sb_idx_omega   = QtWidgets.QSpinBox(); self.sb_idx_omega.setRange(0, 31); self.sb_idx_omega.setValue(self.idx_omega)
        self.sb_idx_current = QtWidgets.QSpinBox(); self.sb_idx_current.setRange(0, 31); self.sb_idx_current.setValue(self.idx_current)
        self.sb_idx_vbus    = QtWidgets.QSpinBox(); self.sb_idx_vbus.setRange(0, 31); self.sb_idx_vbus.setValue(self.idx_vbus)
        self.sb_idx_torque  = QtWidgets.QSpinBox(); self.sb_idx_torque.setRange(0, 31); self.sb_idx_torque.setValue(self.idx_torque)
        self.btn_apply_topic= QtWidgets.QPushButton("Uygula / Yeniden Bağlan")
        self.btn_apply_topic.clicked.connect(self._apply_topic_indices)
        f1.addRow("Topic", self.le_topic)
        f1.addRow("ω index", self.sb_idx_omega)
        f1.addRow("I index", self.sb_idx_current)
        f1.addRow("V index", self.sb_idx_vbus)
        f1.addRow("T index", self.sb_idx_torque)
        f1.addRow(self.btn_apply_topic)
        v.addWidget(grp_topic)

        # Electrical / mechanical params
        grp_params = QtWidgets.QGroupBox("Motor / Aktüatör Parametreleri")
        f2 = QtWidgets.QFormLayout(grp_params)
        self.dsb_kv = QtWidgets.QDoubleSpinBox(); self.dsb_kv.setRange(1, 10000); self.dsb_kv.setValue(self.kv); self.dsb_kv.setSuffix(" rpm/V")
        self.dsb_kt = QtWidgets.QDoubleSpinBox(); self.dsb_kt.setDecimals(5); self.dsb_kt.setRange(0, 10); self.dsb_kt.setValue(self.kt); self.dsb_kt.setSuffix(" Nm/A")
        self.cb_lock_kt = QtWidgets.QCheckBox("Kt = 9.5493 / Kv"); self.cb_lock_kt.setChecked(self.kt_lock)
        self.dsb_ratio = QtWidgets.QDoubleSpinBox(); self.dsb_ratio.setRange(1, 200); self.dsb_ratio.setValue(self.default_ratio)
        self.dsb_gear_eff = QtWidgets.QDoubleSpinBox(); self.dsb_gear_eff.setRange(0.1, 1.0); self.dsb_gear_eff.setSingleStep(0.01); self.dsb_gear_eff.setValue(self.gear_eff)
        self.dsb_r = QtWidgets.QDoubleSpinBox(); self.dsb_r.setRange(0.0, 10.0); self.dsb_r.setSingleStep(0.001); self.dsb_r.setValue(self.r_phase); self.dsb_r.setSuffix(" Ω")
        self.dsb_l = QtWidgets.QDoubleSpinBox(); self.dsb_l.setRange(0.0, 0.1); self.dsb_l.setSingleStep(1e-6); self.dsb_l.setDecimals(6); self.dsb_l.setValue(self.l_phase); self.dsb_l.setSuffix(" H")
        self.cb_fixed_v = QtWidgets.QCheckBox("Sabit DC Bus V kullan"); self.cb_fixed_v.setChecked(self.fixed_vbus_enabled)
        self.dsb_vbus = QtWidgets.QDoubleSpinBox(); self.dsb_vbus.setRange(0, 120); self.dsb_vbus.setValue(self.fixed_vbus); self.dsb_vbus.setSuffix(" V")
        self.dsb_ma = QtWidgets.QSpinBox(); self.dsb_ma.setRange(1, 200); self.dsb_ma.setValue(self.ma_window)
        self.dsb_t_nom = QtWidgets.QDoubleSpinBox(); self.dsb_t_nom.setRange(0, 200); self.dsb_t_nom.setValue(self.torque_rated); self.dsb_t_nom.setSuffix(" Nm")
        self.dsb_t_peak = QtWidgets.QDoubleSpinBox(); self.dsb_t_peak.setRange(0, 500); self.dsb_t_peak.setValue(self.torque_peak); self.dsb_t_peak.setSuffix(" Nm")

        self.dsb_kv.valueChanged.connect(self._kv_changed)
        self.cb_lock_kt.toggled.connect(self._lock_changed)
        self.dsb_t_nom.valueChanged.connect(self._limits_changed)
        self.dsb_t_peak.valueChanged.connect(self._limits_changed)

        f2.addRow("Kv", self.dsb_kv)
        f2.addRow("Kt", self.dsb_kt)
        f2.addRow(self.cb_lock_kt)
        f2.addRow("Dişli Oranı", self.dsb_ratio)
        f2.addRow("Dişli Verimi", self.dsb_gear_eff)
        f2.addRow("Faz Direnci R", self.dsb_r)
        f2.addRow("Faz Endüktansı L", self.dsb_l)
        f2.addRow(self.cb_fixed_v)
        f2.addRow("Sabit DC V", self.dsb_vbus)
        f2.addRow("Hareketli Ortalama (n)", self.dsb_ma)
        f2.addRow("Nominal Tork Çizgisi", self.dsb_t_nom)
        f2.addRow("Tepe Tork Çizgisi", self.dsb_t_peak)
        v.addWidget(grp_params)

        # Controls
        grp_ctrl = QtWidgets.QGroupBox("Kontrol")
        h = QtWidgets.QHBoxLayout(grp_ctrl)
        self.btn_pause = QtWidgets.QPushButton("Pause/Resume")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._toggle_pause)
        self.btn_rec = QtWidgets.QPushButton("REC CSV")
        self.btn_rec.setCheckable(True)
        self.btn_rec.toggled.connect(self._toggle_rec)
        h.addWidget(self.btn_pause); h.addWidget(self.btn_rec)
        v.addWidget(grp_ctrl)

        # Live readouts
        grp_stats = QtWidgets.QGroupBox("Anlık Değerler")
        g = QtWidgets.QGridLayout(grp_stats)
        self.lbl_rate = QtWidgets.QLabel("Rate: – Hz")
        self.lbl_speed = QtWidgets.QLabel("Speed: – rpm")
        self.lbl_torque = QtWidgets.QLabel("Torque: – Nm")
        self.lbl_curr = QtWidgets.QLabel("Current: – A")
        self.lbl_vbus = QtWidgets.QLabel("Bus V: – V")
        self.lbl_pwr = QtWidgets.QLabel("Mech P: – W")
        self.lbl_eff = QtWidgets.QLabel("Eff: – %")
        g.addWidget(self.lbl_rate, 0, 0, 1, 2)
        g.addWidget(self.lbl_speed,1,0); g.addWidget(self.lbl_torque,1,1)
        g.addWidget(self.lbl_curr, 2,0); g.addWidget(self.lbl_vbus, 2,1)
        g.addWidget(self.lbl_pwr,  3,0); g.addWidget(self.lbl_eff,  3,1)
        v.addWidget(grp_stats)

        v.addStretch(1)
        return panel

    # ---------- Handlers ----------
    def _apply_topic_indices(self):
        try:
            new_topic = self.le_topic.text().strip()
            self.idx_omega   = self.sb_idx_omega.value()
            self.idx_current = self.sb_idx_current.value()
            self.idx_vbus    = self.sb_idx_vbus.value()
            self.idx_torque  = self.sb_idx_torque.value()

            if new_topic != self.topic:
                # unsubscribe & resubscribe
                if self.sub is not None:
                    self.destroy_subscription(self.sub)
                self.topic = new_topic
                qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
                self.sub = self.create_subscription(Float32MultiArray, self.topic, self.listener_callback, qos)
                self.get_logger().info(f"Re-subscribed to {self.topic}")
        except Exception as e:
            self.get_logger().warn(f"Topic apply error: {e}")

    def _kv_changed(self, val):
        self.kv = float(val)
        if self.cb_lock_kt.isChecked():
            self.kt = kt_from_kv(self.kv)
            self.dsb_kt.blockSignals(True)
            self.dsb_kt.setValue(self.kt)
            self.dsb_kt.blockSignals(False)

    def _lock_changed(self, checked):
        if checked:
            # eşitle ve kilitleme etkisi
            self._kv_changed(self.dsb_kv.value())

    def _limits_changed(self, _):
        self.torque_rated = self.dsb_t_nom.value()
        self.torque_peak  = self.dsb_t_peak.value()
        for ln, pos in [
            (self.vline_t_rated_1, self.torque_rated),
            (self.vline_t_rated_2, self.torque_rated),
            (self.vline_t_rated_3, self.torque_rated),
            (self.vline_t_peak_1,  self.torque_peak),
            (self.vline_t_peak_2,  self.torque_peak),
            (self.vline_t_peak_3,  self.torque_peak),
        ]:
            ln.setPos(pos)

    def _toggle_pause(self, paused):
        self.plot_paused = paused

    def _toggle_rec(self, on):
        if on and not self.recording:
            fname = time.strftime("bldc_log_%Y%m%d_%H%M%S.csv")
            try:
                self.csv_fh = open(fname, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_fh)
                self.csv_writer.writerow(["t(s)","omega(rad/s)","rpm","torque(Nm)","current(A)","vbus(V)","mech_power(W)","eff(%)"])
                self.recording = True
                self.get_logger().info(f"Recording CSV -> {fname}")
            except Exception as e:
                self.get_logger().error(f"CSV open error: {e}")
                self.btn_rec.setChecked(False)
        elif not on and self.recording:
            try:
                self.recording = False
                if self.csv_fh:
                    self.csv_fh.flush(); self.csv_fh.close()
                self.csv_fh = None; self.csv_writer = None
                self.get_logger().info("Recording stopped.")
            finally:
                pass

    def _spin_ros_once(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as e:
            self.get_logger().warn(f"spin_once: {e}")

    # ---------- Data Path ----------
    def listener_callback(self, msg: Float32MultiArray):
        try:
            data = msg.data
            # Güvenli erişim
            if max(self.idx_omega, self.idx_current, self.idx_vbus, self.idx_torque) >= len(data):
                return

            omega  = float(data[self.idx_omega])     # rad/s
            current= float(data[self.idx_current])   # A
            v_in   = float(data[self.idx_vbus])      # V (komut veya gerçek bus)
            torque = float(data[self.idx_torque])    # Nm (output)

            if self.cb_fixed_v.isChecked():
                v_in = float(self.dsb_vbus.value())

            mech_power = max(0.0, omega * torque)    # W (rad/s * Nm)
            elec_power = max(1e-6, v_in * abs(current))

            eff = 100.0 * min(2.0, mech_power / elec_power)  # %

            t = time.monotonic()
            self.ts_data.append(t)
            self.torque_data.append(torque)
            self.speed_data.append(omega)
            self.current_data.append(current)
            self.vbus_data.append(v_in)
            self.power_data.append(mech_power)
            self.eff_data.append(eff)

            # msg rate ölçümü
            self.last_rx_times.append(t)

            # CSV
            if self.recording and self.csv_writer:
                self.csv_writer.writerow([t, omega, omega*RPM_PER_RADPS, torque, current, v_in, mech_power, eff])

        except Exception as e:
            self.get_logger().warn(f"Hata: {e}")

    # ---------- Helpers ----------
    def _moving_avg(self, arr, n):
        if n <= 1 or len(arr) < n:
            return list(arr)
        out = []
        s = 0.0
        q = collections.deque()
        for x in arr:
            q.append(float(x)); s += float(x)
            if len(q) > n:
                s -= q.popleft()
            out.append(s / len(q))
        return out

    def update_plots(self):
        # Anlık hız ve istatistikler
        now = time.monotonic()
        # Rate
        rate = 0.0
        while self.last_rx_times and now - self.last_rx_times[0] > 1.0:
            self.last_rx_times.popleft()
        if len(self.last_rx_times) >= 2:
            dt = (self.last_rx_times[-1] - self.last_rx_times[0]) / max(1, (len(self.last_rx_times)-1))
            if dt > 0:
                rate = 1.0 / dt
        self.lbl_rate.setText(f"Rate: {rate:0.1f} Hz")

        if not self.torque_data:
            return

        rpm_list = [w * RPM_PER_RADPS for w in self.speed_data]
        n_ma = int(self.dsb_ma.value())
        rpm_ma    = self._moving_avg(rpm_list, n_ma)
        torque_ma = self._moving_avg(self.torque_data, n_ma)
        power_ma  = self._moving_avg(self.power_data, n_ma)
        eff_ma    = self._moving_avg(self.eff_data, n_ma)

        # Live readouts
        self.lbl_speed.setText(f"Speed: {rpm_list[-1]:0.1f} rpm")
        self.lbl_torque.setText(f"Torque: {self.torque_data[-1]:0.2f} Nm")
        self.lbl_curr.setText(f"Current: {self.current_data[-1]:0.2f} A")
        self.lbl_vbus.setText(f"Bus V: {self.vbus_data[-1]:0.1f} V")
        self.lbl_pwr.setText(f"Mech P: {self.power_data[-1]:0.1f} W")
        self.lbl_eff.setText(f"Eff: {self.eff_data[-1]:0.1f} %")

        if self.plot_paused:
            return

        # Scatter/Maps (X: torque)
        if len(torque_ma) > 1:
            self.curve_speed.setData(torque_ma, rpm_ma)
            self.curve_power.setData(torque_ma, power_ma)
            self.curve_eff.setData(torque_ma, eff_ma)

        # Time series (relative time axis)
        t0 = self.ts_data[0]
        t_rel = [t - t0 for t in self.ts_data]
        self.ts_curve_speed.setData(t_rel, rpm_ma)
        self.ts_curve_torque.setData(t_rel, torque_ma)
        self.ts_curve_current.setData(t_rel, self._moving_avg(self.current_data, n_ma))
        self.ts_curve_vbus.setData(t_rel, self._moving_avg(self.vbus_data, n_ma))

    # ---------- App Run ----------
    def run(self):
        sys.exit(self.app.exec_())

def main(args=None):
    rclpy.init(args=args)
    node = MotorPlotNode()
    try:
        node.run()
    finally:
        if node.csv_fh:
            node.csv_fh.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
