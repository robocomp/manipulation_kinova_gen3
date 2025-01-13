import sys
import time
from re import search
from threading import Thread
import queue
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QTextEdit,
    QLineEdit,
    QVBoxLayout,
    QWidget,
    QPushButton,
    QHBoxLayout,
    QScrollArea,
    QLabel
)
from PySide6.QtCore import QObject, Signal, Qt, QProcess, QEvent
from parser import Parser

class ChatThread(Thread, QObject):
    new_message = Signal(str, str)
    def __init__(self, parser, command_queue):
        Thread.__init__(self)
        QObject.__init__(self)
        self.parser = parser
        self.running = True
        self.command_queue = command_queue

    def run(self):
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)  # Non-blocking get with timeout
                print("Command received:", command)
                if command.lower() == "exit":
                    self.running = False
                    break

                result = self.parser.execute_command(command_str=command)
                self.new_message.emit(result, "green")
            except queue.Empty:
                pass

    def stop(self):
        self.running = False

class ChatWindow(QWidget):
    def __init__(self, parent, model):
        super().__init__(parent)
        self.parent = parent
        self.model = model
        self.parser = Parser(model)
        self.command_queue = queue.Queue()

        # A scroll area that will hold a "container" widget
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        self.container = QFrame()
        self.container_layout = QVBoxLayout(self.container)
        self.container_layout.setAlignment(Qt.AlignBottom)  # Key: align everything at bottom
        self.scroll_area.setWidget(self.container)
        self.scroll_area.setStyleSheet("background-color: rgb(218, 255, 202)")

        # Input field (QLineEdit)
        self.input_field = QLineEdit(self.parent)
        self.input_field.returnPressed.connect(self.send_input_to_thread)
        self.input_field.setFocus()
        # Install the event filter on the input field
        self.input_field.installEventFilter(self)

        # Send button (QPushButton)
        self.send_button = QPushButton("Restart", self.parent)
        self.send_button.clicked.connect(self.exit_application)

        # Input Layout
        input_layout = QHBoxLayout()
        input_layout.addWidget(self.input_field)
        input_layout.addWidget(self.send_button)

        # Selected element label on top of the scroll area
        selected_element_title = QLabel("Selected:")
        self.selected_element_label = QLabel("None")
        sel_elements_layout = QHBoxLayout()
        sel_elements_layout.addWidget(selected_element_title)
        sel_elements_layout.addWidget(self.selected_element_label)

        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.addLayout(sel_elements_layout)
        main_layout.addWidget(self.scroll_area)
        main_layout.addLayout(input_layout)
        self.parent.setLayout(main_layout)

        # Create and start the chat thread
        self.chat_thread = ChatThread(self.parser, self.command_queue)
        self.chat_thread.new_message.connect(self.append_to_chat)
        self.chat_thread.start()

    def eventFilter(self, source, event):
        if event.type() == QEvent.KeyPress and source is self.input_field:
            if event.key() == Qt.Key_Up:
                previous_command = self.parser.get_previous_command()
                if previous_command:
                    self.input_field.setText(previous_command)
            elif event.key() == Qt.Key_Down:
                next_command = self.parser.get_next_command()
                if next_command:
                    self.input_field.setText(next_command)
        return super().eventFilter(source, event)

    def send_input_to_thread(self):
        """Sends the user's input to the chat thread."""
        command = self.input_field.text()
        self.input_field.clear()
        self.append_message(f"> {command}", "blue", Qt.AlignmentFlag.AlignRight)
        self.command_queue.put(command)

    def exit_application(self):
        """Restart the application"""
        self.chat_thread.stop()
        QProcess.startDetached(sys.executable, sys.argv)
        QApplication.quit()

    def append_message(self, text, color, alignment):
        """Appends a message to the chat display with specified color and alignment."""

        label = QLabel(text)
        label.setStyleSheet("color: " + color + ";")
        if alignment == Qt.AlignmentFlag.AlignRight:
            label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.container_layout.addWidget(label, 0, Qt.AlignmentFlag.AlignBottom)

        # Force the scroll area to scroll to bottom
        vsb = self.scroll_area.verticalScrollBar()
        vsb.setValue(vsb.maximum())

    def append_to_chat(self, text, color="black"):
        """Appends text to the chat display with specified color."""
        self.append_message(text, color, Qt.AlignmentFlag.AlignLeft) # Display responses on the right

    def closeEvent(self, event):
        self.chat_thread.stop()
        self.chat_thread.join()
        event.accept()
