import threads.AADLThreadFunctionsSupport as tfs
from comments.Comment import Comment
import datetime
from threads.AADLThread import isMainThread
from libraries.Library import Library, ROSBase_TF_Interface
from datatypes.Type import ROSBase_PointerToTransformationFrames
from variables.Variable import Variable
import os


class AADLProcess:
    def __init__(self, process, system_root, system):
        self.process = process
        self.system_root = system_root
        self.system = system
        self.namespace = self.system.namespace

        ##############################
        ### PARAMETRI DELLA CLASSE ###
        ##############################
        self.aadl_node_name = tfs.getName(self.process)
        self.class_name = self.aadl_node_name.title()
        self.node_name = self.aadl_node_name.title()
        self.type = tfs.getType(self.process)
        self.class_libraries = []
        self.class_internal_vars = []
        self.class_constants = []
        self.class_public_methods = []
        self.class_private_methods = []

        ############################
        ### TRANSFORMATION FRAME ###
        ############################
        self.node_uses_tf = False

        ######################
        ### INTERNAL STATE ###
        ######################
        self.node_configuration = None

        ####################
        ### SOURCE FILES ###
        ####################
        self.source_text_files = []

        #########################################
        ### PARAMETRI PER GESTIONE DEL SYSTEM ###
        #########################################
        self.generated = False

        # Contiene la lista dei threads che compongono ogni processo
        self.threads = []

    #######################
    ### GET MAIN THREAD ###
    #######################

    def getMainThread(self):
        for t in self.threads:
            if isMainThread(t.type):
                return t
        return None

    ############################
    ### TRANSFORMATION FRAME ###
    ############################
    def setUsesTransformationFrame(self, state):
        if self.node_uses_tf:
            return
        self.node_uses_tf = state

    def addTransformationFrameComponent(self):
        if self.node_uses_tf:
            tf_var = Variable(self)
            tf_var.setName("tf")
            tf_var.setType(ROSBase_PointerToTransformationFrames(self))
            self.addInternalVariable(tf_var)
            self.getMainThread().constructor.addMiddleCode("{} = new node_base::TransformationFrames();"
                                                           .format(tf_var.name))

            self.addLibrary(ROSBase_TF_Interface())

    ###############
    ### LIBRARY ###
    ###############
    def addLibrary(self, _lib, add_to_cmake=True, add_to_package_xml=True):
        for l in self.class_libraries:
            if l.isEqualTo(_lib):
                return False

        self.class_libraries.append(_lib)

        if add_to_cmake:
            self.system.cmake_list.addPackage(_lib)

        if add_to_package_xml:
            self.system.package_xml.addDependency(_lib)
        return True

    def removeLibrary(self, _lib):
        try:
            self.class_libraries.remove(_lib)
            self.system.cmake_list.removePackage(_lib)
            self.system.package_xml.removeDependency(_lib)
            return True
        except ValueError:
            return False

    #######################################
    ### INTERNAL STATE AND SOURCE FILES ###
    #######################################

    def setNodeConfiguration(self, node_config):
        self.node_configuration = node_config
        self.addSourceFile(self.node_configuration)

    def addSourceFile(self, source_file):
        self.source_text_files.append(source_file)

        source_import = Library()
        source_import.setPath(source_file.getSourceLibraryPath())

        self.addLibrary(source_import, add_to_cmake=False, add_to_package_xml=False)

    def getSourceFile(self, source_file_name):
        (name, ext) = os.path.splitext(source_file_name)
        for s in self.source_text_files:
            if s.file_name == name:
                return s
        return None

    #################
    ### VARIABLES ###
    #################

    def addInternalVariable(self, _var):
        for v in self.class_internal_vars:
            if v.isEqualTo(_var):
                return False
        self.class_internal_vars.append(_var)

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
            self.class_public_methods.append(_method)

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
        # Res > 1 nel caso in cui ci sia la node configuration, altrimenti no
        res = self.compareList(self.class_libraries, another_process.class_libraries)
        if res > 1 and self.node_configuration:
            return False
        elif res != 0 and self.node_configuration is None:
            return False

        # Ha le stesse variabili interne
        res = self.compareList(self.class_internal_vars, another_process.class_internal_vars)
        if res != 0:
            return False

        # Ha le stesse costanti
        # La costante NODE_NAME è da saltare, poichè contiene il nome del nodo che sarebbe diverso
        # in ogni caso per due nodi identici con le stesse caratteristiche
        res = self.compareList(self.class_constants, another_process.class_constants)
        if res != 0:
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
        return len(self.class_private_methods) > 0

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

        text = "Node {}\n".format(self.node_name)
        text += "File auto-generated on {}".format(generated_on)

        disclaimer = Comment()
        disclaimer.setComment(text)

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
        code += "class {} : public node_base::ROSNode {{\n".format(self.class_name)

        # INSTESTAZIONE METODI PRIVATI
        if self.hasPrivateData():
            code += "private:\n"

            for m in self.class_private_methods:
                if m.namespace:
                    code += "\t{}\n".format(m.generateInterface())

        for var in self.class_internal_vars:
            code += "\t{}\n".format(var.generateCode())

        # INSTESTAZIONE METODI PUBBLICI
        if len(self.class_public_methods) > 0:
            code += "public:\n"

            for m in self.class_public_methods:
                if m.namespace:
                    code += "\t{}\n".format(m.generateInterface())

        # Chiudo l'intestazione della classe
        code += "};"
        code += "\n\n"

        ##############################
        ### METODI IMPLEMENTAZIONE ###
        ##############################

        for m in self.class_private_methods + self.class_public_methods:
            code += m.generateCode()

        return code
