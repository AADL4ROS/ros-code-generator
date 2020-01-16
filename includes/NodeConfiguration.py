from includes.Include import Include
from comments.Comment import Comment
from libraries.Library import Library, ROSBase_TF_Interface
from structs.Struct import Struct


class NodeConfiguration(Include):
    def __init__(self, _associated_class):
        super().__init__(_associated_class, "{}_configuration".format(_associated_class.node_name))

        self.structs = []

        self.has_parameters = False
        self.has_variables = False

    def examineStructForLibraris(self, s):
        for v in s.variables:
            if isinstance(v, Struct):
                self.examineStructForLibraris(v)
            else:
                if v.type.library:
                    self.addLibrary(v.type.library)

    def addStruct(self, struct):
        self.structs.append(struct)
        self.examineStructForLibraris(struct)

    def generateCode(self):
        if self.associated_class.node_uses_tf:
            self.addLibrary(ROSBase_TF_Interface())

        code = ""

        # Include
        code += "#ifndef _{}_H\n".format(self.file_name.upper())
        code += "#define _{}_H\n".format(self.file_name.upper())

        c = Comment()
        c.setComment("Auto-generated Internal State")
        code += c.generateCode()

        code += "#include \"node_base/Configuration.h\"\n"

        for l in self.libraries:
            code += l.generateCode()

        code += "\n"

        # Aggiungo le struct
        for s in self.structs:
            code += "".join([gen_s + "\n" for gen_s in s.generateCode().split("\n")])

        # Da qui in poi il codice è SEMPRE UGUALE, non ci saranno mai modifiche
        # da fare in maniera dinamica, non ha senso generarlo, lo si inserisce così

        if self.has_parameters:
            code += "typedef std::shared_ptr < const Parameters > Parameters_ptr;\n"

        if self.has_variables:
            code += "typedef std::shared_ptr < Variables > Variables_ptr;\n"

        code += "class InternalState: node_base::InternalStateBase {\n"
        code += "public:\n"

        if self.has_variables:
            code += "\tVariables_ptr vars() {\n"
            code += "\t\treturn std::static_pointer_cast < Variables > (_vars);\n"
            code += "\t};\n"

        code += "\n"

        if self.has_parameters:
            code += "\tParameters_ptr params() const {\n"
            code += "\t\treturn std::static_pointer_cast < const Parameters > (_params);\n"
            code += "\t};\n"

            code += "\n"

        code += "\tvoid initialize (node_base::ParametersBase * p = NULL) {\n"

        if self.has_parameters:
            code += "\t\t_params = std::make_shared < const Parameters > (*static_cast < Parameters * > (p));\n"

        if self.has_variables:
            code += "\t\t_vars = std::make_shared < Variables > ();\n"

        code += "\t}\n"
        code += "};\n"

        # Chiusura finale visto che si è aperto con degli ifndef
        code += "#endif"

        return code
