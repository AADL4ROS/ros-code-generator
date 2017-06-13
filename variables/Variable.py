from CObject import CObject

class Variable(CObject):
    def __init__(self, _associated_class = None ):
        super().__init__( _associated_class )
        self.type           = None
        self.name           = None
        self.default_value  = None

    def setType(self, _type):
        self.type = _type

    def setName(self, _name):
        self.name = _name

    def setDefaultValue(self, _default_value):
        self.default_value = _default_value

    def generateCode(self):
        default = ""
        if self.default_value != None:
            default = " = {}".format( self.default_value )

        return "{} {}{};".format( self.type.generateCode(), self.name, default )