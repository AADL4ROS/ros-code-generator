from comments.Comment import Comment
import datetime

class AADLProcess():
    def __init__(self, process):
        self.process = process

        ##############################
        ### PARAMETRI DELLA CLASSE ###
        ##############################
        self.class_name             = None
        self.node_name              = None
        self.class_libraries        = []
        self.class_params           = []
        self.class_vars             = []
        self.class_constants        = []
        self.class_public_methods   = []
        self.class_private_methods  = []

        # Contiene la lista dei threads che compongono ogni processo
        self.threads        = []

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
        if len(self.class_private_methods) > 0:
            code += "private:\n"

            for m in self.class_private_methods:
                code += "\t{}\n".format( m.generateInterface() )

        # INSTESTAZIONE METODI PUBBLICI
        if len(self.class_public_methods) > 0:
            code += "public:\n"

            for m in self.class_public_methods:
                code += "\t{}\n".format( m.generateInterface() )

        # Chiudo l'intestazione della classe
        code += "};"

        print( code )
        return code

