import logging
log = logging.getLogger("root")

import os

import threads.AADLThreadFunctionsSupport as tfs

import FolderTreeFunctions as folderTree
import systems.SystemsManager as sm

from cmakelists.CMakeLists import CMakeLists
from package.PackageXML import PackageXML

class System():
    def __init__(self, system_root, namespace = None):

        # Caso di system veri e propri che contengono process
        if system_root != None:
            self.system_root    = system_root
            self.namespace      = tfs.getNamespace(system_root)
            # Log dell'inizio della generazione del system
            log.info("System {}".format(tfs.getType(system_root)))

        # Caso di system che sono composti da data e basta e quindi
        # non hanno una vera e propria system root
        else:
            self.system_root = None
            self.namespace = namespace
            log.info("Data system {}".format(self.namespace))

        # Creo i file CMakeLists e PackageXML
        self.cmake_list     = CMakeLists( self )
        self.package_xml    = PackageXML( self )

        # Resetto la struttura di cartelle (eliminando anche tutti i file) solamente alla prima generazione
        # del namespace, in tutte le altre i file non vengono eliminati
        self.system_folder = folderTree.createFolderTreeForSystem(self.namespace,
                                                             delete=(not sm.isSystemAlreadyReset(self.namespace)))
        sm.addResetSystem(self.namespace)

        # Contiene tutti i nodi da generare
        self.nodes = []

        # Contiene tutti i messaggi da generare
        self.messages = []

        # Contiene tutti i servizi da generare
        self.services = []

    #####################
    ### SYSTEM FOLDER ###
    #####################

    def setSystemFolder(self, system_folder):
        self.system_folder = system_folder

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

    ############
    ### NODI ###
    ############

    def addNode(self, node):
        self.nodes.append(node)

    def isAlreadyGenerated(self, node):
        for n in self.nodes:
            if n.generated and n.isEqualTo( node ):
                return (True, n.class_name)

        return (False, None)

    def saveNode(self, node):
        (already_generated, generated_node_name) = self.isAlreadyGenerated(node)
        if already_generated:
            log.info("Node {} already generated as {}.".format(node.class_name, generated_node_name))

        # Cartella SRC del system
        src_folder = folderTree.getSrcFolderForSystemFolder(self.system_folder)

        filename = "{}.cpp".format(node.class_name)
        source_output_path = os.path.join(src_folder, filename)

        with open(source_output_path, 'w+') as file:
            file.write(node.generateCode())

        log.info("Node {} generated in {}.".format(node.class_name, source_output_path))

        # Setto il nodo come generato
        node.generated = True

        # Dopo il salvataggio aggiungo il nodo al file CMake
        self.cmake_list.addExecutable({
            'name': node.type,
            'path': folderTree.getOnlySrcPathForNode(filename)
        })

    #############################
    ### MESSAGES AND SERVICES ###
    #############################

    def saveMessagesAndServices(self):
        # Salvo i services
        for s in self.services:
            s.saveServiceSRV()

        # Salvo i messagges
        for m in self.messages:
            m.saveMessageMSG()

    ##################################
    ### CMAKELISTS AND PACKAGE XML ###
    ##################################

    def saveCMakeListsAndPackageXML(self):
        self.cmake_list.saveCMakeList()
        self.package_xml.savePackageXML()

    #######################
    ### GENERATE SYSTEM ###
    #######################

    def generateSystem(self):
        # Salvo i nodi
        for n in self.nodes:
            self.saveNode(n)

        # Salvo i messaggi ed i servizi
        self.saveMessagesAndServices()

        # Salvo i file CMakeLists e Package XML
        self.saveCMakeListsAndPackageXML()