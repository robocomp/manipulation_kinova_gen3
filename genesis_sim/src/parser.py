from pyparsing import CaselessKeyword, Word, alphas, nums, Optional, Suppress, Group, ZeroOrMore

class Parser:
    def __init__(self):
        self.grammar = self.define_grammar()

    def define_grammar(self):
        """Defines the grammar for the command language."""

        # Basic elements
        SELECT = CaselessKeyword("select")
        SHOW = CaselessKeyword("show")
        MODEL = CaselessKeyword("model")
        REACH = CaselessKeyword("reach")
        GRASP = CaselessKeyword("grasp")
        LIFT = CaselessKeyword("lift")
        OBJECT_TYPE = Word(alphas)  # For simplicity, allow only letters in type
        OBJECT_NAME = Word(alphas + nums)  # Allow letters and numbers for the name

        # Commands
        show_model_command = Group(SHOW + MODEL)("show_model")
        select_command = Group(SELECT + OBJECT_TYPE + OBJECT_NAME)("select")
        show_object_command = Group(SHOW + OBJECT_TYPE + OBJECT_NAME)("show_object")
        reach_command = Group(OBJECT_TYPE + OBJECT_NAME + REACH)("reach")
        grasp_command = Group(OBJECT_TYPE + OBJECT_NAME + GRASP)("grasp")
        lift_command = Group(OBJECT_TYPE + OBJECT_NAME + LIFT)("lift")

        # Overall command structure
        command = show_model_command | select_command | show_object_command | reach_command | grasp_command | lift_command

        return command