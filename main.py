import os
import importlib
import log
logger = log.setup_custom_logger("root")

# Threads
from threads.AADLThread import AADLThreadMapping
from threads.AADLThread import AADLThreadType

from threads.AADLThreadFunctionsSupport import areThreadsEqual

from threads.AADLProcess import AADLProcess

import datetime
import XMLTags

from lxml import etree

#################
### PARAMETRI ###
#################

ocarina_ros_path    = "../ocarina-ros/"
xml_filename        = "container.tlk_lis_ever_xml.xml"
json_filename       = "tag_ever_xml.json"

#############
### SETUP ###
#############

# Elimino i vecchi file generati
dir             = os.path.dirname(__file__)
output_folder   = os.path.join(dir, "src")

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

def creaNuovoThread( process, thread, classname, associated_class ):
    if classname == None:
        return None

    # Importo il modulo che contiene la tipologia di thread che voglio aggiungere
    thread_module   = importlib.import_module("threads." + classname)

    # Ottengo la classe che gestisce quel particolare thread
    thread_class    = getattr(thread_module, classname)

    # Creo una nuova istanza della classe relativa al thread e lancio quindi la procedura di
    # creazione effettiva del codice
    new_thread      = thread_class(process, thread, associated_class)

    (status, desc) = new_thread.populateData()

    if not status:
        logger.error("Error during the generation of thread: {}\n\t{}".format( new_thread.name, desc ))
        return None

    return new_thread

def saveNode(p, source):
    output_folder       = os.path.join(dir, "src")
    filename            = "{}.cpp".format( p.class_name )
    source_output_path  = os.path.join(output_folder, filename)

    with open(source_output_path, 'w+') as file:
        file.write(source)

###################
### LETTURA XML ###
###################

# Leggo il file XML generato dal backend ever_xml
tree = etree.parse(ocarina_ros_path + xml_filename)

# Ottengo la root del system preso in considerazione
root = tree.getroot()

# Ricerco tutti i processi all'interno del system
processes = root.findall("./" +
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

        gen_main_thread = creaNuovoThread(  process,
                                            main_thread,
                                            AADLThreadMapping.NAME_TO_CLASS.get(AADLThreadType.MAIN_THREAD, "Generic"),
                                            p)
        p.threads.append(gen_main_thread)

        for thread in threads:
            name        = (thread.find(XMLTags.tags['TAG_NAME']).text).lower()
            type        = (thread.find( XMLTags.tags['TAG_TYPE'] ).text).lower()
            namespace   = (thread.find(XMLTags.tags['TAG_NAMESPACE']).text).lower()

            if namespace == "ros":
                new_thread = creaNuovoThread(   process,
                                                thread,
                                                AADLThreadMapping.NAME_TO_CLASS.get(type, None),
                                                p)
                if new_thread != None:
                    p.threads.append( new_thread )

        saveNode(p, p.generateCode())