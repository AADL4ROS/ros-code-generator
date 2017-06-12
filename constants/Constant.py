from CObject import CObject

class Constant(CObject):
    def __init__(self, _associated_class = None ):
        super().__init__( _associated_class )
        self.name   = None
        self.value  = None

    def setName(self, _name):
        self.name = _name

    def setValue(self, _value):
        self.value = _value

    def generateCode(self):
        return "#define {} \"{}\"".format( self.name, self.value )