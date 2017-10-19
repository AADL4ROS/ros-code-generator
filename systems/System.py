import logging
log = logging.getLogger("root")

import XMLTags
import os

import threads.AADLThreadFunctionsSupport as tfs

import FolderTreeFunctions as folderTree
import systems.SystemsManager as sm

from cmakelists.CMakeLists import CMakeLists
from package.PackageXML import PackageXML
from launchfile.LaunchFile import LaunchFile
from launchfile.Node import Node
import launchfile.LaunchFileFunctionSupport as lfsf

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

        # Variabile che conterrà il launch file
        self.launch_file    = None

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

    ####################
    ###  LAUNCH FILE ###
    ####################

    # Alla fine di questa procedura il launch file sa esattamente quali nodi deve aggiungere e quali no
    def createLaunchFile(self):
        log.info("Start launch file creation for system {}".format(self.namespace))
        self.launch_file = LaunchFile( self )

        # Dopo aver creato il launch file, analizzo tutte le connessioni del system e me le salvo
        system_connections = self.system_root.findall("./" + XMLTags.tags['TAG_CONNECTIONS'] + "/" +
                                                 XMLTags.tags['TAG_CONNECTION'])

        for c in system_connections:
            # Cerco le info <port_info> per ciascuna connessione, in modo
            # da sapere quali sono i processi coinvolti
            process_name_list = lfsf.getInvolvedProcessNamePerConnection(self.system_root, c)
            if process_name_list is None:
                log.warning("Unable to retrieve involved process in connection: {}"
                         .format(c.find(XMLTags.tags['TAG_NAME']).text))
                continue

            for p in process_name_list:
                (sub_category, subcomponent) = lfsf.getSubcomponentByName(self.system_root, p['name'])

                # Se non trovo nulla salto
                if sub_category == None or subcomponent == None:
                    continue

                # Se è un process, allora lo aggiungo alla lista di quelli che dovranno essere gestiti
                if sub_category.lower() == "process":
                    n = Node(subcomponent)
                    status = True
                    hasNode = self.launch_file.hasNode(n)
                    if hasNode:
                        n = self.launch_file.retrieveNode(n.name)

                    status = status and n.addPortForTopicFromPortName(p['port'], c)
                    if not status:
                        log.error("Node {} error in remapping topic name"
                                     .format(p['name']))

                    if status:
                        if not hasNode:
                            self.launch_file.addNode(n)
                            log.info("Node {} added successfully"
                                        .format(p['name']))
                        else:
                            log.info("Node {} updated successfully"
                                        .format(p['name']))
                    else:
                        log.error("Node {} encountered an error and it was not added to the launch file"
                                     .format(p['name']))

        log.info("End launch file creation for system {}".format(self.namespace))

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
            return

        # Cartella SRC del system
        src_folder = folderTree.getSrcFolderForSystemFolder(self.system_folder)

        filename = "{}.cpp".format(node.class_name)
        source_output_path = os.path.join(src_folder, filename)

        with open(source_output_path, 'w+') as file:
            file.write(node.generateCode())

        log.info("Node {} generated in {}.".format(node.class_name, source_output_path))

        # Setto il nodo come generato
        node.generated = True

        # Se il CMakeLists con la chiamata addExecutable ritorna False, allora vuol dire che un
        # eseguibile con quel name (quindi node.type) esiste già. Siccome abbiamo generato il nodo
        # vogliamo anche compilarlo e quindi cerchiamo un name (node.type) che ancora non è stato usato


        # Dopo il salvataggio aggiungo il nodo al file CMake
        added_to_executable = self.cmake_list.addExecutable({
                                    'name': node.type,
                                    'path': folderTree.getOnlySrcPathForNode(filename)
                                })
        index_incremental_name = 2

        while not added_to_executable:
            # Creo un nuovo tipo chiamato {node.type}_{index_incremental_name}
            # fino a che non viene aggiunto al CMakeLists
            node.type = "{}_{}".format(node.type, index_incremental_name)
            index_incremental_name += 1

            added_to_executable = self.cmake_list.addExecutable({
                'name': node.type,
                'path': folderTree.getOnlySrcPathForNode(filename)
            })

        # Se il nodo viene salvato, allora devo anche controllare che il tipo nel CMakeLists e quello
        # richiamato nel launch file coincidano
        n = self.launch_file.retrieveNode(node.aadl_node_name)
        n.type = node.type

        # Genero i file da includere
        if node.node_configuration != None:
            node.node_configuration.saveFile()

        for f in node.source_text_files:
            f.saveFile()

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

    ##################################
    ### CMAKELISTS AND PACKAGE XML ###
    ##################################

    def saveLaunchFile(self):
        if self.launch_file != None:
            self.launch_file.saveLaunchFile()

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

        # Salvo il launch file
        self.saveLaunchFile()