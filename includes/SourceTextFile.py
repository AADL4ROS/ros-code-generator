from includes.Include import Include
from comments.Comment import Comment
from libraries.Library import Library, ROSBase_TF_Interface


class SourceTextFile(Include):
    def __init__(self, _associated_class, file_name):
        super().__init__(_associated_class, file_name)

        self.functions = []

    def addFunction(self, f):
        self.functions.append(f)

    def hasFunctionFromName(self, f_name):
        for f in self.functions:
            if f.function_name == f_name:
                return True
        return False

    def getFunctionFromName(self, f_name):
        for f in self.functions:
            if f.function_name == f_name:
                return f
        return None

    def isEqualTo(self, another_object):

        if self.file_name != another_object.file_name:
            return False

        return True

    def generateCode(self):
        # Subito prima di generare il codice devo aggiungere la libreria con la node configuration
        # se serve ed è usata
        if self.associated_class.node_configuration != None:
            lib = Library()
            lib.setPath( self.associated_class.node_configuration.getSourceLibraryPath() )
            self.addLibrary( lib )

        if self.uses_tf:
            self.addLibrary( ROSBase_TF_Interface() )

        code = ""

        for l in self.libraries:
            code += l.generateCode()

        code += "\n"

        for f in self.functions:
            code += "".join([gen_s + "\n" for gen_s in f.generateCode().split("\n")])

        return code