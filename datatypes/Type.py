from CObject import CObject

# Import per classi già definite
import libraries.Library as lib

class Type(CObject):
    def __init__(self, _associated_class, _namespace = None, _type_name = None ):
        super().__init__( _associated_class )
        self.isConst    = False
        self.namespace  = _namespace
        self.type_name  = _type_name
        self.library    = None
        self.afterTypeName = ""

    def setAfterTypeName(self, _text):
        self.afterTypeName = _text

    def setConst(self, _const = True):
        self.isConst = _const

    def setNamespace(self, _namespace):
        self.namespace = _namespace

    def setTypeName(self, _type_name):
        self.type_name = _type_name

    def setLibrary(self, _library):
        self.library = _library
        self.associated_class.addLibrary( _library )

    def removeLibrary(self):
        self.associated_class.removeLibrary( self.library )
        self.library = None

    def shouldImportLibrary(self):
        return (self.library != None)

    def generateCode(self):
        const_str = ""
        if self.isConst:
            const_str = "const "

        if ( self.namespace == None ):
            return "{}{}{}".format(const_str, self.type_name, self.afterTypeName)
        else:
            return "{}{}::{}{}".format(const_str, self.namespace, self.type_name, self.afterTypeName)


class Nothing(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setTypeName( "" )

class Void(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setTypeName( "void" )

class Int(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setTypeName( "int" )

class Bool(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setTypeName( "bool" )

class Double(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setTypeName( "double" )

class String(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std" )
        self.setTypeName( "string" )
        self.setLibrary( lib.StdMsgs_String() )

class Char(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setTypeName( "char" )

class ROS_TimerEvent(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setConst(_const = True)
        self.setNamespace("ros")
        self.setTypeName( "TimerEvent&" )

class ROS_Timer(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace("ros")
        self.setTypeName( "Timer" )

class ROS_Publisher(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace("ros")
        self.setTypeName( "Publisher" )

class ROS_Subscriber(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace("ros")
        self.setTypeName( "Subscriber" )