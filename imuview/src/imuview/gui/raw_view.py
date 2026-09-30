"""Raw MTData2 view: table of the decoded blocks (XDI id, name, value) and the newest byte
chunks as hex. No 3D. Imports Qt."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHeaderView,
    QPlainTextEdit,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
)

from .theme import mono_font  # noqa: E402


class RawPanel(QSplitter):
    def __init__(self, controller):
        super().__init__(Qt.Vertical)
        self.controller = controller
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["XDI", "Блок", "Значення"])
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.hex = QPlainTextEdit()
        self.hex.setReadOnly(True)
        self.hex.setFont(mono_font(9))
        self.hex.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.addWidget(self.table)
        self.addWidget(self.hex)
        self.setSizes([300, 200])

    def refresh(self):
        c = self.controller
        mid = c.active
        if mid is None:
            self.table.setRowCount(0)
            self.hex.setPlainText("")
            return
        t = c.view_time()
        rows = c.store.raw_table(mid, t)
        self.table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            for col, text in enumerate(row):
                item = self.table.item(r, col)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(r, col, item)
                if item.text() != text:
                    item.setText(text)
        self.hex.setPlainText("\n".join(c.store.raw_hex(mid, t, 14)))
