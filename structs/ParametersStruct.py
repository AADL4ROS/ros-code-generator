from structs.Struct import Struct

class ParametersStruct(Struct):
    def __init__(self, _associated_class):
        super().__init__( _associated_class, "Parameters")

        self.super_class = "ros_base::ParametersBase"