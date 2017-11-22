from CObject import CObject
from comments.Comment import Comment
from libraries.Library import Library

class SourceTextFunction(CObject):
    def __init__(self, _associated_class, source_text_file, function_name):
        super().__init__(_associated_class)

        self.source_text_file = source_text_file

        self.function_name = function_name
        self.function_type = None
        self.function_parameters = []

        self.uses_tf = False

    def setTF(self, tf):
        self.source_text_file.uses_tf = tf
        self.uses_tf = tf

    def setFunctionType(self, function_type):
        self.function_type = function_type

        if self.function_type.namespace != None:
            lib = Library()
            lib.setPath("{}/{}.h".format(self.function_type.namespace, self.function_type.type_name))
            self.source_text_file.addLibrary(lib)

    def addFunctionParameter(self, param):
        # Se il parametro già esiste, allora non lo aggiugno.
        for p in self.function_parameters:
            if param.isEqualTo(p):
                return

        self.function_parameters.append(param)

        if param.type.namespace != None:
            lib = Library()
            lib.setPath("{}/{}.h".format(param.type.namespace, param.type.type_name))
            self.source_text_file.addLibrary(lib)

    def addServiceReqAndRes(self, req, res):
        self.function_parameters.append(req)
        self.function_parameters.append(res)

    def isEqualTo(self, another_object):

        if self.function_name != another_object.function_name:
            return False

        if not self.function_type.isEqualTo( another_object.function_type ):
            return False

        return True

    def generateInlineCode(self):
        code = ""

        code += "{}(".format(self.function_name)

        if self.associated_class.node_configuration != None:
            if self.associated_class.node_configuration.has_variables:
                code += " is.vars(),"

            if self.associated_class.node_configuration.has_parameters:
                code += " is.params(),"

            for p in self.function_parameters:
                param_name = p.name.replace("&", "")
                code += " {},".format(param_name)

            if self.uses_tf:
                code += " tf,"

            # Rimuovo l'ultima virgola
            code = code[:-1]

        code += ")"

        return code

    def generateCode(self):
        code = ""

        code += "{} {}(".format( self.function_type.generateCode(), self.function_name )

        if self.associated_class.node_configuration != None:

            if self.associated_class.node_configuration.has_variables:
                code += " Variables_ptr v,"

            if self.associated_class.node_configuration.has_parameters:
                code += " Parameters_ptr p,"

            for p in self.function_parameters:
                code += " {},".format(p.generateCode())

            if self.uses_tf:
                code += " ros_base::TransformationFrames * tf,"

            # Rimuovo l'ultima virgola
            code = code[:-1]

        code += ") {\n"

        # Commento per dire dove inserire il codice
        code += "\n"
        c = Comment()
        c.setComment("Insert here your custom code")
        code += "".join(["\t{}\n".format(c_lin) for c_lin in c.generateCode().split("\n")])

        code += "}\n"

        return code
