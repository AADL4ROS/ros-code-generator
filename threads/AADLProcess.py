import threads.AADLThreadFunctionsSupport as tfs
from comments.Comment import Comment
import datetime
from threads.AADLThread import isMainThread

class AADLProcess():
    def __init__(self, process, system):
        self.process    = process
        self.system     = system
        self.namespace  = tfs.getNamespace(self.system)

        # Riferimento al file CMakeList che verrà generato alla fine
        # della gestione del system
        self.cmake_list     = None
        self.package_xml    = None

        ##############################
        ### PARAMETRI DELLA CLASSE ###
        ##############################
        self.class_name             = tfs.getName( self.process ).title()
        self.node_name              = tfs.getName( self.process ).title()
        self.type                   = tfs.getType( self.process )
        self.class_libraries        = []
        self.class_params           = []
        self.class_vars             = []
        self.class_internal_vars    = []
        self.class_constants        = []
        self.class_public_methods   = []
        self.class_private_methods  = []

        # Contiene la lista di tutti i messaggi custom presenti nel nodo
        self.messages = []

        # Contiene la lista di tutti i services presenti nel nodo
        self.services = []

        # Contiene la lista dei threads che compongono ogni processo
        self.threads = []

    ######################
    ### SET CMAKE LIST ###
    ######################
    def setCMakeList(self, cmake_list):
        self.cmake_list = cmake_list

    #######################
    ### SET PACKAGE XML ###
    #######################
    def setPackageXML(self, package_xml):
        self.package_xml = package_xml

    #######################
    ### GET MAIN THREAD ###
    #######################

    def getMainThread(self):
        for t in self.threads:
            if isMainThread(t.type):
                return t
        return None

    ###################
    ### ADD MESSAGE ###
    ###################
    def addMessage(self, _msg):
        for l in self.messages:
            if l.isEqualTo(_msg):
                return False

        self.messages.append(_msg)

        self.cmake_list.addMessage(_msg)
        return True

    ####################
    ### ADD SERVICES ###
    ####################
    def addService(self, _ser):
        for l in self.services:
            if l.isEqualTo(_ser):
                return False

        self.services.append(_ser)

        self.cmake_list.addService(_ser)
        return True

    ###############
    ### LIBRARY ###
    ###############
    def addLibrary(self, _lib):
        for l in self.class_libraries:
            if l.isEqualTo(_lib):
                return False

        self.class_libraries.append( _lib )
        self.cmake_list.addPackage( _lib )
        self.package_xml.addDependency( _lib )
        return True

    def removeLibrary(self, _lib):
        try:
            self.class_libraries.remove( _lib )
            self.cmake_list.removePackage(_lib)
            self.package_xml.removeDependency( _lib )
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

    def removeInternalVariable(self, _var):
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

    ##################
    ### COMPARISON ###
    ##################

    def elementIsInList(self, elem, list):
        for l in list:
            if elem.isEqualTo(l):
                return True
        return False

    def compareList(self, listA, listB):
        return [self.elementIsInList(l, listB) for l in listA].count(False)

    def isEqualTo(self, another_process):

        # Include le stesse librerie
        res = self.compareList(self.class_libraries, another_process.class_libraries)
        if res != 0:
            return False

        # Ha gli stessi parametri
        res = self.compareList(self.class_params, another_process.class_params)
        if res != 0:
            return False

        # Ha le stesse variabili
        res = self.compareList(self.class_vars, another_process.class_vars)
        if res != 0:
            return False

        # Ha le stesse variabili interne
        res = self.compareList(self.class_internal_vars, another_process.class_internal_vars)
        if res != 0:
            return False

        # Ha le stesse costanti
        # La costante NODE_NAME è da saltare, poichè contiene il nome del nodo che sarebbe diverso
        # in ogni caso per due nodi identici con le stesse caratteristiche
        res = self.compareList(self.class_constants, another_process.class_constants)
        if res > 1:
            return False

        # Per i metodi devo saltare il nome del metodo costruttore, altrimenti due nodi con nomi
        # diversi, ma tutto il resto uguale vengono visti come diversi

        # Ha gli stessi metodi pubblici
        res = self.compareList(self.class_public_methods, another_process.class_public_methods)
        if res > 1:
            return False

        # Ha gli stessi metodi privati
        res = self.compareList(self.class_private_methods, another_process.class_private_methods)
        if res != 0:
            return False

        return True

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

        code += "\n"

        ################
        ### COSTANTI ###
        ################

        for c in self.class_constants:
            code += c.generateCode() + "\n"

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

        #print( code )
        return code