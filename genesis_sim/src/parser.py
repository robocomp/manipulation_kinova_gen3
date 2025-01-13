from token import LEFTSHIFT

from PySide6.QtCore import QObject, Signal, Qt, QProcess, QEvent
from pyparsing import CaselessKeyword, Word, alphas, nums, Optional, Suppress, Group, ZeroOrMore

class Parser(QObject):
    cmd_signal = Signal(object, dict)
    exit_signal = Signal()
    def __init__(self, model):
        super().__init__()
        self.grammar = self.define_grammar()
        self.model = model
        self.command_history = []
        self.history_index = -1

    def define_grammar(self):
        """Defines the grammar for the command language."""

        # Basic elements
        SELECT = CaselessKeyword("select")
        SEL = CaselessKeyword("sel")
        SHOW = CaselessKeyword("show")
        AFFORDANCES = CaselessKeyword("affs")
        MODEL = CaselessKeyword("model")
        REACH = CaselessKeyword("reach")
        GRASP = CaselessKeyword("grasp")
        LIFT = CaselessKeyword("lift")
        IDLE = CaselessKeyword("idle")
        STOP = CaselessKeyword("stop")
        SEARCH = CaselessKeyword("search")
        OPEN = CaselessKeyword("open")
        CLOSE = CaselessKeyword("close")
        HELP = CaselessKeyword("help")
        EXIT = CaselessKeyword("exit")
        DOWN = CaselessKeyword("down")
        UP = CaselessKeyword("up")
        LEFT = CaselessKeyword("left")
        RIGHT = CaselessKeyword("right")
        OBJECT_TYPE = Word(alphas)  # For simplicity, allow only letters in type
        OBJECT_NAME = Word(alphas + nums)  # Allow letters and numbers for the name

        # Commands
        model_command = Group(MODEL)("model")
        show_model_command = Group(SHOW + MODEL)("show_model")
        show_affs_command = Group(SHOW + AFFORDANCES)("show_affs")
        select_command = Group((SELECT | SEL) + OBJECT_TYPE + OBJECT_NAME)("select")
        show_object_command = Group(SHOW + OBJECT_TYPE + OBJECT_NAME)("show_object")
        reach_command = Group(REACH)("reach")
        grasp_command = Group(OBJECT_TYPE + OBJECT_NAME + GRASP)("grasp")
        lift_command = Group(LIFT)("lift")
        search_command = Group(SEARCH)("search")
        idle_command = Group(IDLE | STOP)("idle")
        open_command = Group(OPEN)("open")
        close_command = Group(CLOSE)("close")
        exit_command = Group(EXIT)("exit")
        help_command = Group(HELP)("help")
        down_command = Group(DOWN)("down")
        up_command = Group(UP)("up")
        left_command = Group(LEFT)("left")
        right_command = Group(RIGHT)("right")

        # Overall command structure
        command = ((
                    show_model_command | model_command | select_command | show_object_command | reach_command |
                    grasp_command | lift_command | show_affs_command  | search_command) | idle_command | open_command |
                    close_command | help_command | exit_command | down_command | up_command | left_command | right_command)

        return command

    def execute_command(self, command_str):
        """Parses and executes a single command."""
        try:
            result = self.grammar.parseString(command_str, parseAll=True)
            command_type = result[0].getName()
            self.command_history.append(command_str)
            self.history_index = len(self.command_history)
            #
            match command_type:
                case "model" | "show_model":
                    return self.model.print_model()
                case "select":
                    _, object_type, object_name = result[0]
                    self.model.last_selected_element = self.model.get_element_by_index(int(object_name))
                    self.cmd_signal.emit(self.model.gen3.aff_select_cube, {"cube": self.model.last_selected_element})
                    return "selected: " + object_type + " " + object_name
                case "show_affs":
                    return self.model.gen3.print_affordances()
                case "reach":
                     self.cmd_signal.emit(self.model.gen3.aff_reach_cube, {"cube": self.model.last_selected_element})
                     return "reaching cube " + str(self.model.get_index_of_element(self.model.last_selected_element))
                case "search":
                     self.cmd_signal.emit(self.model.gen3.aff_exploring_from_above, {})
                     return "searching"
                case "idle":
                    self.cmd_signal.emit(self.model.gen3.aff_idle, {})
                    return "idle"
                case "lift":
                    self.cmd_signal.emit(self.model.gen3.aff_lift, {"cube": self.model.last_selected_element})
                    return "lifting arm "
                case "open":
                    self.cmd_signal.emit(self.model.gen3.aff_open_gripper, {})
                    return "opening gripper"
                case "close":
                    self.cmd_signal.emit(self.model.gen3.aff_close_gripper, {})
                    return "closing gripper"
                case "exit":
                    self.exit_signal.emit()
                    return "exiting"
                case "help":
                    return ("Commands: \n" +
                            "   \033[1mmodel\033[0m  (shows elems in model)\n" +
                            "   select cube N\n" +
                            "   show affs (shows available affordances)\n" +
                            "   reach (approaches arm to elem)\n" +
                            "   grasp ()\n" +
                            "   lift (raises arm above elem)\n" +
                            "   open (opens gripper)\n" +
                            "   close (closes gripper)\n" +
                            "   search (moves up an around)\n" +
                            "   idle (does nothing)\n")
                case _:
                     print(f"Error: Unknown command type '{command_type}'.")

        except Exception as e:
            print(f"Error: Invalid command: {e}")
            return "Invalid command"

    def get_previous_command(self):
        """Returns the previous command in the history."""
        if self.history_index > 0:
            self.history_index -= 1
            return self.command_history[self.history_index]
        return None

    def get_next_command(self):
        """Returns the next command in the history."""
        if self.history_index < len(self.command_history) - 1:
            self.history_index += 1
            return self.command_history[self.history_index]
        return None
######################################################################################3

