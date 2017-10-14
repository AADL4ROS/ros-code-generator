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

    def isEqualTo(self, another_constant):
        if self.name != another_constant.name:
            return False

        if self.value != another_constant.value:
            return False

        return True

    def generateCode(self):
        return "#define {} \"{}\"".format( self.name, self.value )