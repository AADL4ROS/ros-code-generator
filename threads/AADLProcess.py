import threads.AADLThreadFunctionsSupport as tfs
from comments.Comment import Comment
import datetime
from threads.AADLThread import AADLThreadType

class AADLProcess():
    def __init__(self, process):
        self.process = process

        ##############################
        ### PARAMETRI DELLA CLASSE ###
        ##############################
        self.class_name             = tfs.getName( self.process ).title()
        self.node_name              = tfs.getName( self.process ).title()
        self.class_libraries        = []
        self.class_params           = []
        self.class_vars             = []
        self.class_internal_vars    = []
        self.class_constants        = []
        self.class_public_methods   = []
        self.class_private_methods  = []

        # Contiene la lista dei threads che compongono ogni processo
        self.threads        = []

    #######################
    ### GET MAIN THREAD ###
    #######################

    def getMainThread(self):
        for t in self.threads:
            if t.type == AADLThreadType.MAIN_THREAD:
                return t
        return None

    ###############
    ### LIBRARY ###
    ###############
    def addLibrary(self, _lib):
        if _lib not in self.class_libraries:
            self.class_libraries.append( _lib )

    def removeLibrary(self, _lib):
        try:
            self.class_libraries.remove( _lib )
            return True
        except ValueError:
            return False

    #################
    ### PARAMTERS ###
    #################
    def addParameter(self, _param):
        self.class_params.append( _param )

    def removeParameter(self, _param):
        try:
            self.class_params.remove( _param )
            return True
        except ValueError:
            return False

    #################
    ### VARIABLES ###
    #################
    def addVariable(self, _var):
        self.class_vars.append(_var)

    def addInternalVariable(self, _var):
        self.class_internal_vars.append(_var)

    def removeVariable(self, _var):
        try:
            self.class_vars.remove(_var)
            return True
        except ValueError:
            return False

    def removenternalVariable(self, _var):
        try:
            self.class_internal_vars.remove(_var)
            return True
        except ValueError:
            return False

    #################
    ### CONSTANTS ###
    #################
    def addConstant(self, _const):
        self.class_constants.append(_const)

    def removeConstant(self, _const):
        try:
            self.class_constants.remove(_const)
            return True
        except ValueError:
            return False

    ###############
    ### METHODS ###
    ###############

    def addPublicMethod(self, _method):
        if _method not in self.class_public_methods:
            self.class_public_methods.append( _method )

    def addPrivateMethod(self, _method):
        if _method not in self.class_private_methods:
            self.class_private_methods.append(_method)

    def removePublicMethod(self, _method):
        try:
            self.class_public_methods.remove(_method)
            return True
        except ValueError:
            return False

    def removePrivateMethod(self, _method):
        try:
            self.class_private_methods.remove(_method)
            return True
        except ValueError:
            return False

    ##############
    ### HELPER ###
    ##############

    def hasPrivateData(self):
        return  (len(self.class_private_methods) > 0 or \
                len(self.class_params) > 0 or \
                len(self.class_vars) > 0 )

    #####################
    ### GENERATE CODE ###
    #####################
    def generateCode(self):
        code = ""

        # Genero il codice finale che andrà effettivamente compilato
        # La primissima cosa da fare è generare un commento che dice che il codice è stato auto-generato,
        # dopo si passa al codice vero e proprio.

        today = datetime.datetime.now()
        generated_on = today.strftime("%d/%m/%Y %H:%M:%S")

        text = "Node {}\n".format( self.node_name )
        text += "File auto-generated on {}".format(generated_on)

        disclaimer = Comment()
        disclaimer.setComment( text )

        code += disclaimer.generateCode()

        ################
        ### LIBRERIE ###
        ################

        for l in self.class_libraries:
            code += l.generateCode()

        ################
        ### COSTANTI ###
        ################

        for c in self.class_constants:
            code += c.generateCode()

        ###########################
        ### INTESTAZIONE CLASSE ###
        ###########################

        code += "\n"
        code += "class {} : public ros_base::ROSNode {{\n".format( self.class_name )

        # INSTESTAZIONE METODI PRIVATI
        if self.hasPrivateData():
            code += "private:\n"

            for m in self.class_private_methods:
                if m.namespace != None:
                    code += "\t{}\n".format( m.generateInterface() )

            if len(self.class_params) > 0:
                code += "\tstruct params {\n"
                for param in self.class_params:
                    code += "\t\t{}\n".format( param.generateCode() )

                code +=  "\t} params;\n"

            if len(self.class_vars) > 0:
                code += "\tstruct vars {\n"
                for var in self.class_vars:
                    code += "\t\t{}\n".format( var.generateCode() )

                code +=  "\t} vars;\n"

        for var in self.class_internal_vars:
            code += "\t{}\n".format( var.generateCode() )

        # INSTESTAZIONE METODI PUBBLICI
        if len(self.class_public_methods) > 0:
            code += "public:\n"

            for m in self.class_public_methods:
                if m.namespace != None:
                    code += "\t{}\n".format( m.generateInterface() )

        # Chiudo l'intestazione della classe
        code += "};"
        code += "\n\n";

        ##############################
        ### METODI IMPLEMENTAZIONE ###
        ##############################

        for m in self.class_private_methods + self.class_public_methods:
            code += m.generateCode()

        print( code )
        return code

