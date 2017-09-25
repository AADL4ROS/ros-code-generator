import os
import importlib
import log
logger = log.setup_custom_logger("root")

# Threads
from threads.AADLThread import AADLThreadMapping
from threads.AADLThread import AADLThreadType

from threads.AADLProcess import AADLProcess

import threads.AADLThreadFunctionsSupport as tfs

import datetime
import XMLTags

from lxml import etree

#################
### PARAMETRI ###
#################

# Input
ocarina_ros_path    = "../ocarina-ros/"
xml_filename        = "container2.tlk_lis_ever_xml.xml"
json_filename       = "tag_ever_xml.json"

# Output
dir             = os.path.dirname(__file__)
output_folder   = os.path.join(dir, "src")

#############
### SETUP ###
#############


# Elimino i vecchi file generati
for the_file in os.listdir(output_folder):
    file_path = os.path.join(output_folder, the_file)
    try:
        if os.path.isfile(file_path):
            os.unlink(file_path)
    except Exception as e:
        print(e)

# Salvo il momento della generazione automatica, in modo da poterlo segnare nel file
today           = datetime.datetime.now()
generated_on    = today.strftime("%d/%m/%Y %H:%M:%S")
print( "Avvio generazione: {}".format(generated_on) )

created_threads = []

def creaNuovoThread( system_root, process, thread, classname, associated_class ):
    if classname == None:
        return None

    # Importo il modulo che contiene la tipologia di thread che voglio aggiungere
    thread_module   = importlib.import_module("threads." + classname)

    # Ottengo la classe che gestisce quel particolare thread
    thread_class    = getattr(thread_module, classname)

    # Creo una nuova istanza della classe relativa al thread e lancio quindi la procedura di
    # creazione effettiva del codice
    new_thread      = thread_class(system_root, process, thread, associated_class)

    (status, desc) = new_thread.populateData()

    if not status:
        logger.error("Error during the generation of thread: {}\n\t{}".format( new_thread.name, desc ))
        return None

    return new_thread

def renameNodeClassIfAlreadyExisting(p):
    filename = "{}.cpp".format(p.class_name)
    source_output_path = os.path.join(output_folder, filename)

    name_index = 2
    while os.path.isfile(source_output_path):
        p.class_name += "_{}".format(name_index)
        p.node_name += "_{}".format(name_index)
        filename = "{}.cpp".format(p.class_name)
        source_output_path = os.path.join(output_folder, filename)
        name_index += 1

def saveNode(p):
    filename = "{}.cpp".format(p.class_name)
    source_output_path = os.path.join(output_folder, filename)

    with open(source_output_path, 'w+') as file:
        file.write(p.generateCode())

def generateCodeForSystem(system_root):
    logger.info("System {}".format( tfs.getType( system_root)) )

    # Ricerco tutti i processi all'interno del system
    processes = system_root.findall("./" +
                                        XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                            XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                                "[" + XMLTags.tags['TAG_CATEGORY'] + "='process']")

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

            p = AADLProcess(process)
            renameNodeClassIfAlreadyExisting(p)

            gen_main_thread = creaNuovoThread(  system_root,
                                                process,
                                                main_thread,
                                                AADLThreadMapping.NAME_TO_CLASS.get(AADLThreadType.MAIN_THREAD, "Generic"),
                                                p)
            p.threads.append(gen_main_thread)

            for thread in threads:
                name        = (tfs.getName(thread)).lower()
                type        = (tfs.getType(thread)).lower()
                namespace   = (tfs.getNamespace(thread)).lower()

                if namespace == "ros":
                    new_thread = creaNuovoThread(   system_root,
                                                    process,
                                                    thread,
                                                    AADLThreadMapping.NAME_TO_CLASS.get(type, None),
                                                    p)
                    if new_thread != None:
                        p.threads.append( new_thread )

            saveNode(p)


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
systems         = [system_root]

while len(systems) > 0:
    s = systems.pop(0)

    generateCodeForSystem(s)

    # Mi cerco eventuali system dentro ad altri system. Questo serve nel caso in cui un
    # system sia usato come subcomponents di un altro system. La cosa è ricorsiva, poiché
    # di volta in volta la system_root diventa il system considerato.
    # La prima visita si fa comunque alla system root, dopo si passa a visitare ricorsivamente
    # tutti i vari system
    systems.extend( s.findall("./" +
                          XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                          XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                            XMLTags.tags['TAG_SYSTEM'] + "/" +
                          "[" + XMLTags.tags['TAG_CATEGORY'] + "='system']") )
