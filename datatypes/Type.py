from CObject import CObject

# Import per classi già definite
import libraries.Library as lib

class Type(CObject):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.namespace  = None
        self.type_name  = None
        self.library    = None

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
        if ( self.namespace == None ):
            return self.type_name
        else:
            return "{}::{}".format(self.namespace, self.type_name)


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

class String(Type):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.setNamespace( "std" )
        self.setTypeName( "string" )
        self.setLibrary( lib.StdMsgs_String() )