from CObject import CObject

class Variable(CObject):
    def __init__(self, _associated_class = None):
        super().__init__( _associated_class )

        # Oggetto di tipo Type, bisogna chiamare il generate code
        self.type = None

        # Stringa
        self.name = None

    def setType(self, _type):
        self.type = _type

    def setName(self, _name):
        self.name = _name

    def generateCode(self):
        return "{} {}".format(self.type.generateCode(), self.name)