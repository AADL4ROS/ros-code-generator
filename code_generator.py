import importlib
import os

# Threads
from threads.AADLThread import getPythonClassFromAADLThreadType, isMainThread
from threads.AADLProcess import AADLProcess
import threads.AADLThreadFunctionsSupport as tfs

import FolderTreeFunctions as folderTree

import datetime
import XMLTags

from lxml import etree

import systems.SystemsManager as sm
from systems.System import System

import global_filepath
import log

logger = log.setup_custom_logger("root")


def creaNuovoThread(system_root, process, thread, classname, associated_class):
    if not classname:
        return None

    # Importo il modulo che contiene la tipologia di thread che voglio aggiungere
    thread_module = importlib.import_module("threads." + classname)

    # Ottengo la classe che gestisce quel particolare thread
    thread_class = getattr(thread_module, classname)

    # Creo una nuova istanza della classe relativa al thread e lancio quindi la procedura di
    # creazione effettiva del codice
    new_thread = thread_class(system_root, process, thread, associated_class)

    (status, desc) = new_thread.populateData()

    if not status:
        logger.error("Error during the generation of thread: {}\n\t{}".format(new_thread.name, desc))
        return None

    return new_thread


def renameNodeClassIfAlreadyExisting(p, system_folder):
    src_folder = folderTree.getSrcFolderForSystemFolder(system_folder)

    filename = "{}.cpp".format(p.class_name)
    source_output_path = os.path.join(src_folder, filename)

    name_index = 2
    while os.path.isfile(source_output_path):
        p.class_name += "_{}".format(name_index)
        p.node_name += "_{}".format(name_index)
        filename = "{}.cpp".format(p.class_name)
        source_output_path = os.path.join(src_folder, filename)
        name_index += 1


def generateCodeForSystem(system_root, system_parent):
    # Generando il system si generano anche i file CMakeLists e PackageXML
    # Viene generata tutto l'albero delle cartelle e viene resettato se necessario
    namespace = tfs.getNamespace(system_root)

    system = sm.getSystemForNamespace(namespace)
    if system == None:
        system = System(system_root)
        sm.addSystem(system)

    # Se il system non ha un launch file associato e serve crearlo, allora lo creo
    # Il launch file a questo punto è vuoto
    if system.launch_file == None:
        system.createLaunchFile()

        # Se ho un genitore, ovvero un system che mi ha incluso,
        # lo avviso che dovrà includermi
        if system_parent != None:
            if system_parent.launch_file != None:
                system_parent.launch_file.addSubSystem(system.launch_file)

    # Ricerco tutti i processi all'interno del system
    processes = system_root.findall("./" +
                                    XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                    XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                    "[" + XMLTags.tags['TAG_CATEGORY'] + "='process']" +
                                    "[" + XMLTags.tags['TAG_NAMESPACE'] + "='" + system.namespace + "']")

    # Scorro ogni processo. Per ogni processo controllo i subcomponent: in base alle varie tipologie
    # di subcomponent avvio la generazione di diversi nodi ROS
    for process in processes:
        threads = process.findall("./" +
                                  XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                  XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                  "[" + XMLTags.tags['TAG_CATEGORY'] + "='thread']")

        # Cerco il main thread, che formerà la base per tutti gli altri thread.
        main_thread = process.find("./" +
                                   XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                   XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                   "[" + XMLTags.tags['TAG_CATEGORY'] + "='thread']" + "/" +
                                   "[" + XMLTags.tags['TAG_NAME'] + "='main_thread']" + "/" +
                                   "[" + XMLTags.tags['TAG_NAMESPACE'] + "='ros']")
        if main_thread != None:
            type = (tfs.getType(main_thread)).lower()

            p = AADLProcess(process, system_root, system)
            renameNodeClassIfAlreadyExisting(p, system.system_folder)

            gen_main_thread = creaNuovoThread(system_root,
                                              process,
                                              main_thread,
                                              getPythonClassFromAADLThreadType(type),
                                              p)
            p.threads.append(gen_main_thread)

            for thread in threads:
                # name        = (tfs.getName(thread)).lower()
                type = (tfs.getType(thread)).lower()
                namespace = (tfs.getNamespace(thread)).lower()

                if (namespace == "ros" or namespace == "global_state_machine") and not isMainThread(type):
                    new_thread = creaNuovoThread(system_root,
                                                 process,
                                                 thread,
                                                 getPythonClassFromAADLThreadType(type),
                                                 p)
                    if new_thread != None:
                        p.threads.append(new_thread)

            p.addTransformationFrameComponent()
            system.addNode(p)


def startGeneration():
    XMLTags.initialize()

    #################
    ### PARAMETRI ###
    #################

    # Input
    ocarina_ros_path = global_filepath.xml_folder_path
    xml_filename = global_filepath.xml_filename

    #############
    ### SETUP ###
    #############

    # Salvo il momento della generazione automatica, in modo da poterlo segnare nel file
    today = datetime.datetime.now()
    generated_on = today.strftime("%d/%m/%Y %H:%M:%S")
    print("Avvio generazione: {}".format(generated_on))

    ###################
    ### LETTURA XML ###
    ###################

    # Leggo il file XML generato dal backend ever_xml
    tree = etree.parse(os.path.join(ocarina_ros_path, xml_filename))

    # Ottengo la root del system preso in considerazione
    system_root = tree.getroot()

    # Siccome itererò su tutti i vari system disponbili, inzio ad aggiungere
    # la mia system root, dopo mano a mano aggiungerò anche tutti gli altri system su
    # cui fare code-generation
    systems = [{
        'system': system_root,
        'parent': None
    }]

    while len(systems) > 0:
        s = systems.pop(0)

        generateCodeForSystem(s['system'], s['parent'])

        # Mi cerco eventuali system dentro ad altri system. Questo serve nel caso in cui un
        # system sia usato come subcomponents di un altro system. La cosa è ricorsiva, poiché
        # di volta in volta la system_root diventa il system considerato.
        # La prima visita si fa comunque alla system root, dopo si passa a visitare ricorsivamente
        # tutti i vari system
        sub_systems = s['system'].findall("./" +
                                          XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                          XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                          XMLTags.tags['TAG_SYSTEM'] + "/" +
                                          "[" + XMLTags.tags['TAG_CATEGORY'] + "='system']")

        # Io so chi è il genitore dei nodi, mi basta chiedere il system con quel certo
        # namespace al systems manager
        system_parent = sm.getSystemForNamespace(tfs.getNamespace(s['system']))

        for sub_sys in sub_systems:
            systems.append({
                'system': sub_sys,
                'parent': system_parent
            })

    sm.generateAllSystems()
