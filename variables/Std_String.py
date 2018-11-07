from variables.Variable import Variable
from datatypes.Type import String


class Std_String(Variable):
    def __init__(self, _associated_class):
        super().__init__(_associated_class)
        self.setType(String(self.associated_class))
