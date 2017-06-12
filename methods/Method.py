from CObject import CObject

# Import per classi già definite
import libraries.Library as lib

# @TODO: gestione Source Text
class Method(CObject):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.namespace      = None
        self.method_name    = None
        self.return_type    = None
        self.input_params   = []
        self.library        = None
        self.codice         = ""
        self.source_text    = None

    def setLibrary(self, _library):
        self.library = _library
        self.associated_class.addLibrary( _library )

    def removeLibrary(self):
        self.associated_class.removeLibrary( self.library )
        self.library = None

    def shouldImportLibrary(self):
        return (self.library != None)

    def getIntestazione(self):
        namespace = ""
        if self.namespace != None:
            namespace = "{}::".format(self.namespace)

        code = "{} {}{}(".format(self.return_type.generateCode(), namespace, self.method_name)

        params = ", ".join(p.generateCode() for p in self.input_params)
        code += params

        code += ")"
        return code

    def generateInterface(self):
        return "{};".format( self.getIntestazione() )

    def generateCode(self):
        code = "{} (".format( self.getIntestazione() )

        params = ", ".join(p.generateCode() for p in self.input_params)
        code += params

        code += ") {{\n"
        code += self.codice
        code += "\n}}"

        return code