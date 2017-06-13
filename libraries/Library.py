from CObject import CObject

class Library(CObject):
    def __init__(self, _associated_class = None ):
        super().__init__( _associated_class )
        self.path = None

    def setPath(self, _path):
        self.path = _path

    def generateCode(self):
        return "#include \"{}\"\n".format( self.path )

class StdMsgs_String(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "std_msgs/String.h"

class ROSBase_ROSNode(Library):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.path = "ros_base/ROSNode.h"