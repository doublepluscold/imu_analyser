"""Calibration tab: stepper, live 3D preview with the target orientation, stillness readouts,
summary with the found bias / scale and the quality of every step. Imports Qt."""

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QProgressBar,
    QPushButton,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .. import calib_tools as ct
from . import theme
from .plots_view import style_plot
from .pose_preview import PoseView
from .theme import T
from .widgets import Chip, button, card, label, panel
from .wizard import MAX_ACCEL_STD, ORDER, up_error_deg

STEP_NAMES = {
    "level": "Рівно",
    "left": "Лівий бік вниз",
    "right": "Правий бік вниз",
    "nosedown": "Носом донизу",
    "noseup": "Носом догори",
    "back": "Догори дном",
    "gyro": "Гіроскоп",
}
STEP_KEYS = [*ORDER, "gyro"]
G = 9.80665


class StepItem(QPushButton):
    """One row of the stepper: done (green tick), current (yellow), waiting (grey)."""

    def __init__(self, number: int, key: str):
        super().__init__(f"{number}.  {STEP_NAMES[key]}")
        self.setProperty("variant", "flat")
        self.setCursor(Qt.PointingHandCursor)
        self.state_ = None

    def set_state(self, state: str):
        if state == self.state_:
            return
        self.state_ = state
        color = {"done": T.ok, "current": T.accent, "todo": T.text_muted}[state]
        name = "check" if state == "done" else "circle"
        self.setIcon(theme.icon(name, color, 18))
        weight = "600" if state == "current" else "400"
        text = T.text if state != "todo" else T.text_muted
        self.setStyleSheet(
            f"QPushButton {{ text-align: left; color: {text}; font-weight: {weight};"
            f"border: 1px solid {T.accent if state == 'current' else 'transparent'};"
            f"min-height: 36px; max-height: 36px; padding-left: {T.sp3}px; }}"
            f"QPushButton:hover {{ background: {T.bg3}; }}"
        )
        self.setToolTip("Клік - переробити цей крок" if state == "done" else "")


class Tile(QFrame):
    """Compact numeric tile: caption, value in the mono font, a threshold note and an indicator."""

    def __init__(self, caption: str, note: str = ""):
        super().__init__()
        self.setObjectName("card")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(T.sp3, T.sp2, T.sp3, T.sp2)
        lay.setSpacing(0)
        self.caption = label(caption, muted=True)
        self.value = QLabel("-")
        self.value.setFont(theme.mono_font(18, bold=True))
        self.note = label(note, muted=True)
        self.note.setFont(theme.mono_font(8))
        for w in (self.caption, self.value, self.note):
            lay.addWidget(w)

    def set(self, text: str, ok: bool | None):
        color = T.text if ok is None else (T.ok if ok else T.warn)
        self.value.setText(text)
        self.value.setStyleSheet(f"color: {color};")


class CalibrationPage(QWidget):
    def __init__(self, controller, on_message):
        super().__init__()
        self.c, self.on_message = controller, on_message
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(T.sp3)
        root.addWidget(self._stepper())
        self.stack = QStackedWidget()
        self.stack.addWidget(self._intro())
        self.stack.addWidget(self._step_card())
        self.stack.addWidget(self._summary())
        root.addWidget(self.stack, 1)
        self._summary_for = None

    # ----- building -----

    def _stepper(self):
        box = panel()
        box.setFixedWidth(240)
        lay = QVBoxLayout(box)
        lay.setContentsMargins(T.sp3, T.sp3, T.sp3, T.sp3)
        lay.addWidget(label("Кроки", heading=True))
        self.module_label = label("", muted=True)
        lay.addWidget(self.module_label)
        self.items = []
        for i, key in enumerate(STEP_KEYS):
            it = StepItem(i + 1, key)
            it.clicked.connect(lambda _=False, k=i: self._step_clicked(k))
            it.set_state("todo")
            lay.addWidget(it)
            self.items.append(it)
        lay.addStretch(1)
        return box

    def _intro(self):
        box = card()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(T.sp4 * 2, T.sp4 * 2, T.sp4 * 2, T.sp4 * 2)
        lay.addStretch(1)
        lay.addWidget(label("Калібрація акселерометра і гіроскопа", heading=True))
        text = label(
            "Майстер веде по шести положеннях модуля і зупинці для гіроскопа. Постав модуль, "
            "як показано на 3D-превʼю, і натисни Enter. Нерухомість визначається автоматично.",
            muted=True,
        )
        text.setWordWrap(True)
        lay.addWidget(text)
        self.intro_hint = label("", muted=True)
        lay.addWidget(self.intro_hint)
        self.start = button("Почати калібрацію", "primary", "play")
        self.start.setFixedWidth(220)
        self.start.clicked.connect(self.on_start)
        lay.addWidget(self.start)
        lay.addStretch(2)
        return box

    def _step_card(self):
        box = card()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(T.sp4, T.sp4, T.sp4, T.sp4)
        lay.setSpacing(T.sp3)
        top = QHBoxLayout()
        left = QVBoxLayout()
        self.counter = label("", muted=True)
        self.prompt = label("", heading=True)
        self.prompt.setWordWrap(True)
        self.hint = label("Тримай модуль нерухомо, поки смужка не заповниться.", muted=True)
        self.hint.setWordWrap(True)
        self.match_chip = Chip("", "muted")
        left.addWidget(self.counter)
        left.addWidget(self.prompt)
        left.addWidget(self.hint)
        left.addSpacing(T.sp2)
        left.addWidget(self.match_chip, 0, Qt.AlignLeft)
        left.addStretch(1)
        top.addLayout(left, 2)
        self.preview = PoseView(self.c)
        top.addWidget(self.preview, 3)
        lay.addLayout(top, 3)

        row = QHBoxLayout()
        self.still_chip = Chip("нерухомо", "ok")
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        row.addWidget(self.still_chip)
        row.addWidget(self.bar, 1)
        lay.addLayout(row)
        self.message = label("", muted=True)
        lay.addWidget(self.message)

        tiles = QHBoxLayout()
        self.t_norm = Tile("|a|, м/с²", "ціль 9.81 ± 0.5")
        self.t_std = Tile("Розкид, м/с²", f"поріг < {MAX_ACCEL_STD}")
        self.t_up = Tile("Відхилення від цілі, °", "поріг < 12")
        for t in (self.t_norm, self.t_std, self.t_up):
            tiles.addWidget(t)
        lay.addLayout(tiles)
        self.graph = pg.PlotWidget()
        self.graph.setBackground(T.bg1)
        self.graph.setMenuEnabled(False)
        self.graph.setMouseEnabled(False, False)
        self.graph.setFixedHeight(110)
        style_plot(self.graph.getPlotItem())
        self.graph.getPlotItem().setTitle("|a|, м/с²", size="9pt")
        self.curve = self.graph.plot(pen=pg.mkPen(T.info, width=1.5))
        self.graph.addItem(
            pg.InfiniteLine(G, angle=0, pen=pg.mkPen(T.text_muted, style=Qt.DashLine, width=1))
        )
        lay.addWidget(self.graph)

        btns = QHBoxLayout()
        self.back = button("Назад", icon_name="undo-2")
        self.cancel = button("Скасувати", icon_name="x")
        self.ready = button("Готово", "primary", "check")
        self.save = button("Зберегти і застосувати", "primary", "save")
        btns.addWidget(self.back)
        btns.addWidget(self.cancel)
        btns.addStretch(1)
        btns.addWidget(self.save)
        btns.addWidget(self.ready)
        lay.addLayout(btns)
        self.back.clicked.connect(self.on_back)
        self.cancel.clicked.connect(self.on_cancel)
        self.ready.clicked.connect(self.on_enter)
        self.save.clicked.connect(self.on_save)
        return box

    def _summary(self):
        box = card()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(T.sp4, T.sp4, T.sp4, T.sp4)
        lay.setSpacing(T.sp3)
        head = QHBoxLayout()
        head.addWidget(label("Підсумок калібрації", heading=True))
        self.quality = Chip("", "muted")
        head.addWidget(self.quality)
        head.addStretch(1)
        lay.addLayout(head)
        self.params = QGridLayout()
        self.params.setHorizontalSpacing(T.sp4)
        lay.addLayout(self.params)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Крок", "|a|/g−1 до, %", "після, %", "якість"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        lay.addWidget(self.table, 1)
        self.sum_notes = label("", muted=True)
        self.sum_notes.setWordWrap(True)
        lay.addWidget(self.sum_notes)
        btns = QHBoxLayout()
        self.discard = button("Відкинути результат", icon_name="trash-2")
        self.sum_save = button("Зберегти і застосувати", "primary", "save")
        btns.addWidget(self.discard)
        btns.addStretch(1)
        btns.addWidget(self.sum_save)
        lay.addLayout(btns)
        self.discard.clicked.connect(self.on_cancel)
        self.sum_save.clicked.connect(self.on_save)
        return box

    # ----- actions -----

    def on_start(self):
        try:
            self.c.start_wizard()
        except RuntimeError as e:
            self.on_message(str(e))

    def on_enter(self):
        if self.c.wizard and self.c.mode == "calibration":
            self.c.wizard.press_enter()

    def on_back(self):
        if self.c.wizard:
            self.c.wizard.back()

    def on_cancel(self):
        self.c.end_wizard()
        self.c.mode = "calibration"  # stay on this tab
        self._summary_for = None

    def on_save(self):
        try:
            self.c.finish_wizard()
        except Exception as e:
            self.on_message(f"не збережено: {e}")
            return
        self.on_message(self.c.message)
        self.c.end_wizard()
        self.c.mode = "calibration"

    def _step_clicked(self, i: int):
        w = self.c.wizard
        if w is not None and w.status().done_steps[i]:
            w.goto(i)

    # ----- refresh -----

    def refresh(self):
        c, w = self.c, self.c.wizard
        self.module_label.setText(
            f"Модуль: {c.name(c.active)}" if c.active is not None else "Немає даних від модулів"
        )
        if w is None:
            self.stack.setCurrentIndex(0)
            ready = c.active is not None and c.live is not None
            self.start.setEnabled(ready)
            self.intro_hint.setText(
                ""
                if ready
                else "Спочатку під'єднайся до джерела даних (вгорі), щоб з'явився модуль."
            )
            for it in self.items:
                it.set_state("todo")
            return
        st = w.status()
        for i, it in enumerate(self.items):
            state = "done" if st.done_steps[i] else ("current" if i == st.step else "todo")
            it.set_state(state)
        if st.phase == "done":
            self.stack.setCurrentIndex(2)
            self._fill_summary(w)
            return
        self.stack.setCurrentIndex(1)
        self._fill_step(w, st)

    def _fill_step(self, w, st):
        c = self.c
        key = STEP_KEYS[min(st.step, len(STEP_KEYS) - 1)]
        pose = "level" if key == "gyro" else key
        self.counter.setText(f"Крок {st.step + 1} з {st.n_steps}")
        self.prompt.setText(st.prompt)
        placed = self.preview.refresh(pose)
        if placed is None:
            self.match_chip.set("немає орієнтації", "muted")
        elif placed:
            self.match_chip.set("положення збігається", "ok")
        else:
            self.match_chip.set("поверни модуль як на контурі", "info")
        self.still_chip.set(*(("нерухомо", "ok") if st.still else ("рух", "warn")))
        self.bar.setValue(round(st.progress * 1000))
        self.message.setText(st.message)
        norm = st.accel_norm
        self.t_norm.set("-" if norm is None else f"{norm:.3f}",
                        None if norm is None else abs(norm - G) < 0.5)  # fmt: skip
        self.t_std.set("-" if st.accel_std is None else f"{st.accel_std:.3f}",
                       None if st.accel_std is None else st.accel_std < MAX_ACCEL_STD)  # fmt: skip
        pose_now = c.store.pose_at(c.active, None, use_gps=False) if c.active is not None else None
        if pose_now is not None:
            err = up_error_deg(pose_now.q, pose)
            self.t_up.set(f"{err:.1f}", err < 12.0)
        else:
            self.t_up.set("-", None)
        hist = w.history()
        if hist:
            t = np.array([h[0] for h in hist])
            v = np.array([h[1] for h in hist])
            m = t > t[-1] - 10
            self.curve.setData(t[m] - t[-1], v[m])
            self.graph.setXRange(-10, 0, padding=0)
            self.graph.setYRange(G - 1.5, G + 1.5, padding=0)
        last = st.step == st.n_steps - 1
        self.save.setVisible(last)
        self.save.setEnabled(False)  # enabled on the summary, when every step is done
        self.back.setEnabled(st.step > 0)
        waiting = st.phase == "wait_enter"
        self.ready.setEnabled(waiting)
        self.ready.setText("Готово" if waiting else "Збираю…")

    def _fill_summary(self, w):
        r = w.result
        if self._summary_for is r:
            return
        self._summary_for = r
        acc = r["accel"]
        for i in reversed(range(self.params.count())):
            self.params.itemAt(i).widget().setParent(None)
        ok = not acc["errors"]
        worst = acc.get("max_rel_err_after")
        if not ok:
            self.quality.set("помилка підгонки", "danger")
        elif worst is not None and worst > ct.MAX_REL_ERR:
            self.quality.set(f"залишок до {100 * worst:.2f} %", "warn")
        else:
            self.quality.set("якість добра", "ok")
        rows = []
        if ok:
            rows.append(("Зсув b, м/с²", acc["offset_b"], 4))
            rows.append(("Масштаб M (діагональ)", acc["scale_diag"], 5))
        rows.append(("Зсув гіро, °/с", np.degrees(r["gyro_bias"]).tolist(), 4))
        rows.append(("Шум гіро, °/с", np.degrees(r["gyro_std"]).tolist(), 4))
        if r["level"]:
            rows.append(("Вирівнювання (крен, тангаж), °", r["level"]["level_trim_deg"], 3))
        self.params.addWidget(label("", muted=True), 0, 0)
        for j, axis in enumerate("XYZ"):
            head = label(axis, muted=True)
            head.setAlignment(Qt.AlignRight)
            self.params.addWidget(head, 0, 1 + j)
        for i, (name, vals, digits) in enumerate(rows, start=1):
            self.params.addWidget(label(name), i, 0)
            for j, v in enumerate(vals):
                lb = label(f"{v:+.{digits}f}", mono=True)
                lb.setAlignment(Qt.AlignRight)
                self.params.addWidget(lb, i, 1 + j)
        self.params.setColumnStretch(0, 1)
        self.table.setRowCount(0)
        if ok:
            errs = zip(acc["poses"], acc["rel_err_before"], acc["rel_err_after"], strict=True)
            for pose, before, after in errs:
                row = self.table.rowCount()
                self.table.insertRow(row)
                good = abs(after) <= ct.MAX_REL_ERR
                cells = [STEP_NAMES.get(pose, pose), f"{100 * before:+.3f}",
                         f"{100 * after:+.3f}", "добре" if good else "перевір"]  # fmt: skip
                for col, text in enumerate(cells):
                    item = QTableWidgetItem(text)
                    if col:
                        item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                        item.setFont(theme.mono_font(9))
                    if col == 3:
                        item.setForeground(theme.qcolor(T.ok if good else T.warn))
                    self.table.setItem(row, col, item)
        notes = list(acc["errors"])
        for warning in acc["warnings"]:
            if "exactly determined" in warning:  # the tool's English text, said here in Ukrainian
                notes.append(
                    f"{acc['n_poses']} положень на 6 параметрів: підгонка точна, тож залишок "
                    "після калібрації нічого не доводить. Для перевірки зміряй |a| ще в одному "
                    "положенні."
                )
            else:
                notes.append(warning)
        self.sum_notes.setText("\n".join(notes))
        self.sum_save.setEnabled(w.calibration() is not None)
