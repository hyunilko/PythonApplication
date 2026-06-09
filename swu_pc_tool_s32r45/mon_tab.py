#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mon_tab.py

S32R45 Heartbeat Monitor GUI Tab.
"""

from __future__ import annotations

import csv
import glob
import math
import datetime
import os
from collections import deque
from typing import Optional, List

from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QFont
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QFormLayout,
    QPushButton, QLabel, QLineEdit, QSpinBox, QComboBox,
    QFileDialog, QMessageBox, QScrollArea,
)

from mon_receiver import MonReceiver, MonPacket, NUM_RF
from swu_constants import (
    DEFAULT_PCAN_CHANNEL, DEFAULT_BITRATE_FD, DEFAULT_IFG_US, DEFAULT_DEVICE_ID,
)

MAX_POINTS = 300
_RF_COLORS = [QColor('#e74c3c'), QColor('#e67e22'), QColor('#2ecc71'), QColor('#3498db')]
_A53_COLOR = QColor('#8e44ad')


# ---------------------------------------------------------------------------
# QPainter 기반 단순 시계열 차트
# ---------------------------------------------------------------------------
class _LineChart(QWidget):
    """QPainter로 그리는 다중 시리즈 시계열 차트."""

    def __init__(self, title: str = "", parent=None) -> None:
        super().__init__(parent)
        self.title   = title
        self._series: List[dict] = []   # {'label', 'color': QColor, 'x': list, 'y': list}
        self.setMinimumSize(200, 130)
        self.setMaximumHeight(220)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent)

    def set_series(self, series: List[dict]) -> None:
        """series = [{'label': str, 'color': QColor, 'x': [float,...], 'y': [float,...]}]"""
        self._series = series
        self.update()

    # --- internal helpers ---

    @staticmethod
    def _nice_range(vmin: float, vmax: float):
        """y축 보기 좋은 min/max 계산."""
        span = vmax - vmin
        if span == 0:
            span = 1.0
        pad = span * 0.12
        return vmin - pad, vmax + pad

    def paintEvent(self, a0) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        W, H = self.width(), self.height()
        ml, mr, mt, mb = 46, 8, 22, 28   # margins (left, right, top, bottom)
        pw = W - ml - mr                  # plot width
        ph = H - mt - mb                  # plot height

        # ── background ─────────────────────────────────────────
        p.fillRect(0, 0, W, H, QColor('#f5f5f5'))
        p.fillRect(ml, mt, pw, ph, QColor('white'))
        p.setPen(QPen(QColor('#bbb'), 1))
        p.drawRect(ml, mt, pw, ph)

        # ── title ──────────────────────────────────────────────
        font_title = QFont()
        font_title.setPointSize(8)
        font_title.setBold(True)
        p.setFont(font_title)
        p.setPen(QColor('#333'))
        p.drawText(QRectF(ml, 2, pw, mt - 2),
                   Qt.AlignmentFlag.AlignCenter, self.title)

        # ── data range ─────────────────────────────────────────
        all_x, all_y = [], []
        for s in self._series:
            for x, y in zip(s['x'], s['y']):
                if not math.isnan(y):
                    all_x.append(x)
                    all_y.append(y)

        if not all_x:
            p.end()
            return

        x_min, x_max = min(all_x), max(all_x)
        y_raw_min, y_raw_max = min(all_y), max(all_y)
        y_min, y_max = self._nice_range(y_raw_min, y_raw_max)
        x_span = x_max - x_min if x_max != x_min else 1.0
        y_span = y_max - y_min if y_max != y_min else 1.0

        def to_px(x: float, y: float):
            px = ml + (x - x_min) / x_span * pw
            py = mt + ph - (y - y_min) / y_span * ph
            return px, py

        # ── grid (y) ───────────────────────────────────────────
        font_tick = QFont()
        font_tick.setPointSize(7)
        p.setFont(font_tick)
        n_ytick = 4
        for i in range(n_ytick + 1):
            yv = y_min + y_span * i / n_ytick
            _, py = to_px(x_min, yv)
            p.setPen(QPen(QColor('#e0e0e0'), 1, Qt.PenStyle.DashLine))
            p.drawLine(QPointF(ml, py), QPointF(ml + pw, py))
            p.setPen(QColor('#555'))
            p.drawText(QRectF(0, py - 8, ml - 4, 16),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{yv:.1f}")

        # ── x-axis labels ──────────────────────────────────────
        n_xtick = 4
        for i in range(n_xtick + 1):
            xv = x_min + x_span * i / n_xtick
            px, _ = to_px(xv, y_min)
            p.setPen(QColor('#555'))
            p.drawText(QRectF(px - 18, mt + ph + 3, 36, 14),
                       Qt.AlignmentFlag.AlignCenter, f"{xv:.0f}s")

        # ── series lines ───────────────────────────────────────
        for s in self._series:
            pts = [(x, y) for x, y in zip(s['x'], s['y']) if not math.isnan(y)]
            if len(pts) < 2:
                continue
            pen = QPen(s['color'], 1.5)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            p.setPen(pen)
            prev = QPointF(*to_px(*pts[0]))
            for pt in pts[1:]:
                cur = QPointF(*to_px(*pt))
                p.drawLine(prev, cur)
                prev = cur

        # ── legend (top-left inside plot) ──────────────────────
        lx, ly = ml + 4, mt + 4
        for s in self._series:
            p.setPen(QPen(s['color'], 2))
            p.drawLine(QPointF(lx, ly + 5), QPointF(lx + 14, ly + 5))
            p.setPen(QColor('#333'))
            p.setFont(font_tick)
            p.drawText(int(lx + 16), int(ly + 9), s['label'])
            lx += 52

        p.end()


# ---------------------------------------------------------------------------
# 5-chart panel (RF0..3  2×2  +  A53 full-width)
# ---------------------------------------------------------------------------
class _ChartPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        grid = QGridLayout(self)
        grid.setSpacing(4)
        grid.setContentsMargins(2, 2, 2, 2)

        self._rf_charts: List[_LineChart] = []
        for rfe_i in range(NUM_RF):
            c = _LineChart(title=f"RF {rfe_i}  TX Temp (°C)")
            self._rf_charts.append(c)
            grid.addWidget(c, rfe_i // 2, rfe_i % 2)

        self._a53_chart = _LineChart(title="A53 Core Temp (°C)")
        grid.addWidget(self._a53_chart, 2, 0, 1, 2)

    def update_charts(
        self,
        times:     List[float],
        rf_temps:  List[List[float]],   # [sample][rfe]
        a53_temps: List[float],
    ) -> None:
        if not times:
            return

        for rfe_i in range(NUM_RF):
            y = [rf_temps[s][rfe_i] for s in range(len(times))]
            self._rf_charts[rfe_i].set_series([{
                'label': f"RF{rfe_i}",
                'color': _RF_COLORS[rfe_i],
                'x':     list(times),
                'y':     y,
            }])

        self._a53_chart.set_series([{
            'label': 'A53',
            'color': _A53_COLOR,
            'x':     list(times),
            'y':     list(a53_temps),
        }])


# ---------------------------------------------------------------------------
# Monitor Tab
# ---------------------------------------------------------------------------
class MonitorTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._receiver:   Optional[MonReceiver] = None
        self._pkt_count   = 0
        self._start_time: Optional[float] = None
        self._csv_path:   Optional[str]   = None
        self._csv_file    = None
        self._csv_writer  = None

        self._times:     deque = deque(maxlen=MAX_POINTS)
        self._rf_temps:  deque = deque(maxlen=MAX_POINTS)
        self._a53_temps: deque = deque(maxlen=MAX_POINTS)

        self._cleanup_old_csv()
        self._build_ui()

        self._plot_timer = QTimer()
        self._plot_timer.timeout.connect(self._update_charts)

    @staticmethod
    def _cleanup_old_csv() -> None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        for f in glob.glob(os.path.join(script_dir, "mon_heartbeat_*.csv")):
            try:
                os.remove(f)
            except Exception:
                pass

    # -----------------------------------------------------------------------
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(4)

        # ---- Connection ----
        grp_conn = QGroupBox("PCAN Connection")
        form_conn = QFormLayout(grp_conn)

        self.cmb_channel = QComboBox()
        for i in range(1, 9):
            self.cmb_channel.addItem(f"PCAN_USBBUS{i}")
        self.cmb_channel.setCurrentText(DEFAULT_PCAN_CHANNEL)

        self.ed_bitrate = QLineEdit(DEFAULT_BITRATE_FD)

        self.sp_ifg = QSpinBox()
        self.sp_ifg.setRange(0, 100000)
        self.sp_ifg.setValue(DEFAULT_IFG_US)
        self.sp_ifg.setSuffix(" us")

        self.sp_device_id = QSpinBox()
        self.sp_device_id.setRange(0, 7)
        self.sp_device_id.setValue(DEFAULT_DEVICE_ID)
        self.sp_device_id.setToolTip("CAN ID = (DeviceID << 8) | 0x42")

        form_conn.addRow("Channel",    self.cmb_channel)
        form_conn.addRow("Bitrate FD", self.ed_bitrate)
        form_conn.addRow("IFG",        self.sp_ifg)
        form_conn.addRow("DeviceID",   self.sp_device_id)
        root.addWidget(grp_conn)

        self.sp_device_id.valueChanged.connect(self._on_device_id_changed)

        # ---- Controls ----
        ctrl_row = QHBoxLayout()
        self.btn_start = QPushButton("모니터링 시작")
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop  = QPushButton("모니터링 중지")
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_stop.setEnabled(False)
        self.btn_csv   = QPushButton("CSV 저장 경로...")
        self.btn_csv.clicked.connect(self._on_select_csv)
        self.lb_csv    = QLabel("(CSV 저장 안 함)")
        self.lb_csv.setStyleSheet("color: gray;")
        ctrl_row.addWidget(self.btn_start)
        ctrl_row.addWidget(self.btn_stop)
        ctrl_row.addWidget(self.btn_csv)
        ctrl_row.addWidget(self.lb_csv, 1)
        root.addLayout(ctrl_row)

        # ---- Status ----
        status_row = QHBoxLayout()
        self.lb_status    = QLabel("대기 중")
        self.lb_uniq_id   = QLabel("UniqueID: -")
        self.lb_pkt_count = QLabel("패킷: 0")
        self.lb_last_rx   = QLabel("마지막 수신: -")
        for lbl in (self.lb_status, self.lb_uniq_id, self.lb_pkt_count, self.lb_last_rx):
            lbl.setStyleSheet("font-weight: bold;")
        status_row.addWidget(self.lb_status)
        status_row.addWidget(QLabel(" | "))
        status_row.addWidget(self.lb_uniq_id)
        status_row.addWidget(QLabel(" | "))
        status_row.addWidget(self.lb_pkt_count)
        status_row.addWidget(QLabel(" | "))
        status_row.addWidget(self.lb_last_rx)
        status_row.addStretch()
        root.addLayout(status_row)

        # ---- Latest Values ----
        grp_vals = QGroupBox("최신 값")
        vals_h = QHBoxLayout(grp_vals)
        vals_h.setSpacing(4)

        grp_err = QGroupBox("Error State")
        form_err = QFormLayout(grp_err)
        self.lb_error_type = QLabel("-")
        form_err.addRow("error_type", self.lb_error_type)
        self.lb_error_code: List[QLabel] = []
        for i in range(NUM_RF):
            lbl = QLabel("-")
            form_err.addRow(f"error_code[{i}]", lbl)
            self.lb_error_code.append(lbl)
        vals_h.addWidget(grp_err)

        self.lb_rf: List[QLabel] = []
        for rfe_i in range(NUM_RF):
            grp_rfe = QGroupBox(f"RF {rfe_i}")
            form_rfe = QFormLayout(grp_rfe)
            lbl = QLabel("-")
            form_rfe.addRow("TX Temp", lbl)
            self.lb_rf.append(lbl)
            vals_h.addWidget(grp_rfe)

        grp_a53 = QGroupBox("A53 Core")
        form_a53 = QFormLayout(grp_a53)
        self.lb_a53 = QLabel("-")
        form_a53.addRow("Temp", self.lb_a53)
        vals_h.addWidget(grp_a53)

        root.addWidget(grp_vals)

        # ---- Charts ----
        self._charts = _ChartPanel()
        root.addWidget(self._charts, 1)

    # -----------------------------------------------------------------------
    def _on_select_csv(self) -> None:
        ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default = os.path.join(os.getcwd(), f"mon_heartbeat_{ts}.csv")
        path, _ = QFileDialog.getSaveFileName(
            self, "CSV 저장 경로 선택", default, "CSV Files (*.csv)"
        )
        if path:
            self._csv_path = path
            self.lb_csv.setText(os.path.basename(path))
            self.lb_csv.setStyleSheet("color: black;")

    def _on_start(self) -> None:
        # CSV 경로 미선택 시 스크립트 디렉터리에 자동 생성
        if not self._csv_path:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self._csv_path = os.path.join(script_dir, f"mon_heartbeat_{ts}.csv")
            self.lb_csv.setText(os.path.basename(self._csv_path))
            self.lb_csv.setStyleSheet("color: blue;")

        self._csv_file = self._csv_writer = None
        try:
            self._csv_file   = open(self._csv_path, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._csv_file)
            header  = ['uniq_id', 'recv_time', 'tv_sec', 'tv_nsec', 'error_type']
            header += [f'error_code_{i}' for i in range(NUM_RF)]
            header += [f'rfTxTemp_{rfe}' for rfe in range(NUM_RF)]
            header += ['a53CoresTemp']
            self._csv_writer.writerow(header)
            self._csv_file.flush()
        except Exception as e:
            QMessageBox.warning(self, "CSV", f"파일 열기 실패:\n{self._csv_path}\n{e}")
            self._csv_file = self._csv_writer = None
            self._csv_path = None
            self.lb_csv.setText("(CSV 저장 실패)")
            self.lb_csv.setStyleSheet("color: red;")

        self._times.clear()
        self._rf_temps.clear()
        self._a53_temps.clear()
        self._pkt_count  = 0
        self._start_time = None

        self._start_receiver()

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self._plot_timer.start(1000)

    def _start_receiver(self) -> None:
        """현재 설정으로 MonReceiver를 (재)시작한다."""
        if self._receiver:
            self._receiver.request_stop()
            self._receiver.wait(2000)
            self._receiver = None

        channel   = self.cmb_channel.currentText()
        bitrate   = self.ed_bitrate.text().strip()
        ifg_us    = int(self.sp_ifg.value())
        device_id = int(self.sp_device_id.value())

        self._receiver = MonReceiver(channel, bitrate, ifg_us, device_id)
        self._receiver.packet_received.connect(self._on_packet)
        self._receiver.status_changed.connect(self.lb_status.setText)
        self._receiver.error_occurred.connect(self._on_error)
        self._receiver.start()

    def _on_device_id_changed(self) -> None:
        """DeviceID 변경 시 CAN 수신 필터 갱신 및 그래프 초기화."""
        if self._receiver is None:
            return   # 모니터링 중이 아니면 무시

        # 데이터 버퍼 초기화
        self._times.clear()
        self._rf_temps.clear()
        self._a53_temps.clear()
        self._pkt_count  = 0
        self._start_time = None

        # 상태 초기화
        self.lb_uniq_id.setText("UniqueID: -")
        self.lb_pkt_count.setText("패킷: 0")
        self.lb_last_rx.setText("마지막 수신: -")
        for lbl in self.lb_rf:
            lbl.setText("-"); lbl.setStyleSheet("")
        self.lb_a53.setText("-"); self.lb_a53.setStyleSheet("")
        self.lb_error_type.setText("-"); self.lb_error_type.setStyleSheet("")
        for lbl in self.lb_error_code:
            lbl.setText("-"); lbl.setStyleSheet("")

        # 그래프 초기화
        for chart in self._charts._rf_charts:
            chart.set_series([])
        self._charts._a53_chart.set_series([])

        # 새 DeviceID로 receiver 재시작
        self._start_receiver()

    def _on_stop(self) -> None:
        self._plot_timer.stop()
        if self._receiver:
            self._receiver.request_stop()
            self._receiver.wait(3000)
            self._receiver = None
        if self._csv_file:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = self._csv_writer = None
        # 다음 시작 시 새 타임스탬프 파일로 저장되도록 경로 초기화
        self._csv_path = None
        self.lb_csv.setText("(CSV 저장 안 함)")
        self.lb_csv.setStyleSheet("color: gray;")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def _on_error(self, msg: str) -> None:
        self.lb_status.setText(f"오류: {msg}")
        QMessageBox.critical(self, "Monitor Error", msg)
        self._on_stop()

    # -----------------------------------------------------------------------
    def _on_packet(self, pkt: MonPacket) -> None:
        self._pkt_count += 1
        if self._start_time is None:
            self._start_time = pkt.recv_time

        rel_time = pkt.recv_time - self._start_time
        self._times.append(rel_time)
        self._rf_temps.append(pkt.rfTxTemp)
        self._a53_temps.append(pkt.a53CoresTemp)

        self._update_status_labels(pkt)
        self._update_value_labels(pkt)
        self._write_csv(pkt)

    def _update_status_labels(self, pkt: MonPacket) -> None:
        self.lb_pkt_count.setText(f"패킷: {self._pkt_count}")
        self.lb_uniq_id.setText(f"UniqueID: 0x{pkt.uniq_id:08X}")
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        self.lb_last_rx.setText(f"마지막 수신: {now_str}")

    def _update_value_labels(self, pkt: MonPacket) -> None:
        self.lb_error_type.setText(f"0x{pkt.error_type:08X}")
        self.lb_error_type.setStyleSheet(
            "color: red; font-weight: bold;" if pkt.error_type else "color: green;"
        )
        for i, lbl in enumerate(self.lb_error_code):
            lbl.setText(f"0x{pkt.error_code[i]:08X}")
            lbl.setStyleSheet("color: red;" if pkt.error_code[i] else "")

        for rfe_i in range(NUM_RF):
            val = pkt.rfTxTemp[rfe_i]
            lbl = self.lb_rf[rfe_i]
            if math.isnan(val):
                lbl.setText("N/A"); lbl.setStyleSheet("color: gray;")
            elif val <= -900.0:
                lbl.setText(f"{val:.0f} (Fail)"); lbl.setStyleSheet("color: red;")
            else:
                lbl.setText(f"{val:.1f} °C"); lbl.setStyleSheet("")

        val = pkt.a53CoresTemp
        if math.isnan(val):
            self.lb_a53.setText("N/A"); self.lb_a53.setStyleSheet("color: gray;")
        else:
            self.lb_a53.setText(f"{val:.1f} °C"); self.lb_a53.setStyleSheet("")

    def _write_csv(self, pkt: MonPacket) -> None:
        if self._csv_writer is None or self._csv_file is None:
            return
        try:
            row  = [f"0x{pkt.uniq_id:08X}", f"{pkt.recv_time:.3f}", pkt.tv_sec, pkt.tv_nsec, pkt.error_type]
            row += pkt.error_code
            row += [
                "" if math.isnan(pkt.rfTxTemp[rfe]) else f"{pkt.rfTxTemp[rfe]:.3f}"
                for rfe in range(NUM_RF)
            ]
            row += ["" if math.isnan(pkt.a53CoresTemp)
                    else f"{pkt.a53CoresTemp:.3f}"]
            self._csv_writer.writerow(row)
            self._csv_file.flush()
        except Exception as e:
            self.lb_status.setText(f"CSV 저장 오류: {e}")
            self._csv_writer = None
            self._csv_file   = None

    def _update_charts(self) -> None:
        if not self._times:
            return
        self._charts.update_charts(
            list(self._times),
            list(self._rf_temps),
            list(self._a53_temps),
        )


if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    w = MonitorTab()
    w.setWindowTitle("Monitor (Heartbeat) - 단독 실행")
    w.resize(1200, 800)
    w.show()
    sys.exit(app.exec())
