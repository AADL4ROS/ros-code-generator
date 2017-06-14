from CObject import CObject
from comments.Comment import Comment

# Import per classi già definite
import libraries.Library as lib

# @TODO: gestione Source Text
class Method(CObject):
    def __init__(self, _associated_class):
        super().__init__( _associated_class )
        self.namespace          = None
        self.method_name        = None
        self.return_type        = None
        self.input_params       = []
        self.library            = None
        self.codice_at_top      = []
        self.codice_at_middle   = []
        self.codice_at_bottom   = []
        self.source_text        = None

    def setLibrary(self, _library):
        self.library = _library
        self.associated_class.addLibrary( _library )

    def removeLibrary(self):
        self.associated_class.removeLibrary( self.library )
        self.library = None

    def addInputParameter(self, _param):
        self.input_params.append( _param )

    def shouldImportLibrary(self):
        return (self.library != None)

    def addTopCode(self, _codes):
        if isinstance(_codes, str):
            self.codice_at_top.extend( _codes.split("\n") )
        elif isinstance(_codes, CObject):
            self.codice_at_top.append( _codes )

    def addMiddleCode(self, _codes):
        if isinstance(_codes, str):
            self.codice_at_middle.extend( _codes.split("\n") )
        elif isinstance(_codes, CObject):
            self.codice_at_middle.append( _codes )

    def addBottomCode(self, _codes):
        if isinstance(_codes, str):
            self.codice_at_bottom.extend( _codes.split("\n") )
        elif isinstance(_codes, CObject):
            self.codice_at_bottom.append( _codes )

    def getThreadPointer(self):
        namespace = ""

        if self.namespace != None:
            namespace = "{}::".format(self.namespace)

        return "&{}{}".format(namespace, self.method_name)

    def getIntestazione(self, with_namespace = True):
        namespace = ""

        if with_namespace:
            if self.namespace != None:
                namespace = "{}::".format(self.namespace)

        code = "{} {}{}(".format(self.return_type.generateCode(), namespace, self.method_name)

        params = ", ".join(p.generateCode() for p in self.input_params)
        code += params

        code += ")"
        return code

    def generateInterface(self):
        return "{};".format( self.getIntestazione(with_namespace = False) )

    def generateCode(self):
        comment = Comment()
        comment.setComment( "Method {} auto-generated".format(self.method_name) )
        code = comment.generateCode()

        code += "{} {{\n".format( self.getIntestazione(with_namespace = True) )

        for s in self.codice_at_top + self.codice_at_middle + self.codice_at_bottom:
            if isinstance(s, str):
                if len(s) > 0:
                    code += "\t" + s + "\n"
            elif isinstance(s, CObject):
                code += "".join(["\t" + gen_s + "\n" for gen_s in s.generateCode().split("\n")])

        code += "}\n"
        code += "\n"

        return code