from CObject import CObject

class Message(CObject):
    def __init__(self, types, members, _associated_class = None):
        super().__init__(_associated_class)
        self.types      = types
        self.members    = members

    def createCustomMessage(self):
        for m in self.members:
            name = m['name']
            type = m['type']

            #ros_type = getROSDatatypeFromASN1(m['type'])


    def generateCode(self):
        pass