from CObject import CObject
import os

class Library(CObject):
    def __init__(self, _associated_class = None ):
        super().__init__( _associated_class )
        self.path = None

    def setPath(self, _path):
        self.path = _path

    # Funzione usata dal CMakeList
    # Ritorna il nome del package.
    # Es. std_msgs/String.h -> std_msgs
    def getPackageName(self):
        if self.path != None:
            return os.path.dirname(self.path)
        return ""

    def isEqual(self, _other_library):
        return (self.path == _other_library.path)

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