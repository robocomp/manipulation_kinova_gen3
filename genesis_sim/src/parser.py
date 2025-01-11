import sys
import time
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
    QTextBrowser,
    QScrollBar
)
from PySide6.QtCore import QObject, Signal, Qt, QProcess
from PySide6.QtGui import QTextCursor,  QTextBlockFormat
from pyparsing import CaselessKeyword, Word, alphas, nums, Optional, Suppress, Group, ZeroOrMore

class Parser(QObject):
    cmd_select_signal = Signal(object, dict)
    def __init__(self, model):
        super().__init__()
        self.grammar = self.define_grammar()
        self.model = model

    def define_grammar(self):
        """Defines the grammar for the command language."""

        # Basic elements
        SELECT = CaselessKeyword("select")
        SHOW = CaselessKeyword("show")
        AFFORDANCES = CaselessKeyword("affs")
        MODEL = CaselessKeyword("model")
        REACH = CaselessKeyword("reach")
        GRASP = CaselessKeyword("grasp")
        LIFT = CaselessKeyword("lift")
        OBJECT_TYPE = Word(alphas)  # For simplicity, allow only letters in type
        OBJECT_NAME = Word(alphas + nums)  # Allow letters and numbers for the name

        # Commands
        model_command = Group(MODEL)("model")
        show_model_command = Group(SHOW + MODEL)("show_model")
        show_affs_command = Group(SHOW + AFFORDANCES)("show_affs")
        select_command = Group(SELECT + OBJECT_TYPE + OBJECT_NAME)("select")
        show_object_command = Group(SHOW + OBJECT_TYPE + OBJECT_NAME)("show_object")
        reach_command = Group(OBJECT_TYPE + OBJECT_NAME + REACH)("reach")
        grasp_command = Group(OBJECT_TYPE + OBJECT_NAME + GRASP)("grasp")
        lift_command = Group(OBJECT_TYPE + OBJECT_NAME + LIFT)("lift")

        # Overall command structure
        command = show_model_command | model_command | select_command | show_object_command | reach_command | grasp_command | lift_command | show_affs_command

        return command

    def execute_command(self, command_str):
        """Parses and executes a single command."""
        try:
            result = self.grammar.parseString(command_str, parseAll=True)
            command_type = result[0].getName()
            #
            match command_type:
                case "model" | "show_model":
                    return self.model.print_model()
                case "select":
                    _, object_type, object_name = result[0]
                    #self.model.gen3.get_affordance("aff_select_cube" + object_type)
                    cube = self.model.get_element_by_index(int(object_name))
                    self.cmd_select_signal.emit(self.model.gen3.aff_select_cube, {"cube": cube})
                    return "selected: " + object_type + " " + object_name
                case "show_affs":
                    return self.model.gen3.print_affordances()
                # case "reach":
                #     _, object_name, _ = result[0]
                #     self.model.reach(object_name)
                # case "grasp":
                #     _, object_name, _ = result[0]
                #     self.model.grasp(object_name)
                # case "lift":
                #     _, object_name, _ = result[0]
                #     self.model.lift(object_name)
                # case _:
                #     print(f"Error: Unknown command type '{command_type}'.")

        except Exception as e:
            print(f"Error: Invalid command: {e}")

######################################################################################3

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

class ChatWindow:
    def __init__(self, parent, model):
        self.parent = parent
        self.model = model
        self.parser = Parser(model)
        self.command_queue = queue.Queue()

        # Chat display (QTextEdit)
        self.chat_display = QTextBrowser(self.parent)
        self.chat_display.setReadOnly(True)
        self.chat_display.setStyleSheet("background-color: lightgray;")

        # Input field (QLineEdit)
        self.input_field = QLineEdit(self.parent)
        self.input_field.returnPressed.connect(self.send_input_to_thread)

        # Send button (QPushButton)
        self.send_button = QPushButton("Restart", self.parent)
        self.send_button.clicked.connect(self.exit_application)

        # Layout
        input_layout = QHBoxLayout()
        input_layout.addWidget(self.input_field)
        input_layout.addWidget(self.send_button)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.chat_display)
        main_layout.addLayout(input_layout)

        self.parent.setLayout(main_layout)

        # Create and start the chat thread
        self.chat_thread = ChatThread(self.parser, self.command_queue)
        self.chat_thread.new_message.connect(self.append_to_chat)
        self.chat_thread.start()

        # Timer for simulating received messages (remove in real implementation)
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.fake_response)
        # self.timer.start(2000)  # Simulate receiving a message every 2 seconds

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
        cursor = self.chat_display.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)

        # Create a block format and set the alignment
        block_format = QTextBlockFormat()
        block_format.setAlignment(alignment)

        # Insert the block with the specified alignment
        cursor.insertBlock(block_format)

        # Insert the HTML-formatted text
        html = f'<font color="{color}">{text}</font>'
        cursor.insertHtml(html)

        # Move the cursor to the end and make sure it's visible
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self.chat_display.setTextCursor(cursor)
        self.chat_display.ensureCursorVisible()

    def append_to_chat(self, text, color="black"):
        """Appends text to the chat display with specified color."""
        self.append_message(text, color, Qt.AlignmentFlag.AlignLeft) # Display responses on the right

    def closeEvent(self, event):
        self.chat_thread.stop()
        self.chat_thread.join()
        event.accept()

    def fake_response(self):
        command = "This is a fake response"
        self.append_message(f"> {command}", "green", Qt.AlignmentFlag.AlignLeft)