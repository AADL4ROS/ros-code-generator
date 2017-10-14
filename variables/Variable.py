from CObject import CObject

class Variable(CObject):
    def __init__(self, _associated_class = None ):
        super().__init__( _associated_class )
        self.type           = None
        self.name           = None
        self.default_value  = None
        self.is_function_param = False

    def setIsParameter(self, _param = True):
        self.is_function_param = _param

    def setType(self, _type):
        self.type = _type

    def setName(self, _name):
        self.name = _name

    def setDefaultValue(self, _default_value):
        self.default_value = _default_value

    def hasDefaultValue(self):
        return (self.default_value != None)

    def isEqualTo(self, another_variable):
        # Stesso nome
        if self.name != another_variable.name:
            return False

        # Stesso valore di default
        if self.default_value != another_variable.default_value:
            return False

        # Stesso tipo
        if not self.type.isEqualTo(another_variable.type):
            return False

        return True

    def generateCode(self):
        default = ""
        if self.default_value != None:
            default = " = {}".format( self.default_value )

        punto_e_virgola = ";"
        if self.is_function_param:
            punto_e_virgola = ""

        return "{} {}{}{}".format( self.type.generateCode(), self.name, default, punto_e_virgola )