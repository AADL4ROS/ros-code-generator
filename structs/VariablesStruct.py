from structs.Struct import Struct

class VariablesStruct(Struct):
    def __init__(self, _associated_class):
        super().__init__( _associated_class, "Variables")
        self.has_constructor = True

        self.super_class = "ros_base::VariablesBase"