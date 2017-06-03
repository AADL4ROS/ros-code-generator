import importlib
import logging as log

# Threads
from threads.AADLThread import AADLThreadMapping;

import datetime
import json

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

# Salvo il momento della generazione automatica, in modo da poterlo segnare nel file
today           = datetime.datetime.now()
generated_on    = today.strftime("%d/%m/%Y %H:%M:%S")
print( "Avvio generazione: {}".format(generated_on) )

# Leggo il file JSON generato dal backend di Ocarina che contiene le mappature fra elementi e nomi
# nel file XML
with open(ocarina_ros_path + json_filename) as data_file:
    tags = json.load( data_file )

# Stampo a schermo i tag per controllo
for tag in tags:
    print("{}: {}".format(tag, tags[tag]))

created_threads = []

def creaNuovoThread( process, thread, classname ):
    # Importo il modulo che contiene la tipologia di thread che voglio aggiungere
    thread_module   = importlib.import_module("threads." + classname)

    # Ottengo la classe che gestisce quel particolare thread
    thread_class    = getattr(thread_module, classname)

    # Creo una nuova istanza della classe relativa al thread e lancio quindi la procedura di
    # creazione effettiva del codice
    new_thread      = thread_class(process, thread, tags)

    (status, error_desc) = new_thread.generate_code()
    if not status:
        log.error("Errore durante la generazione del codice")
        log.error(error_desc)

    # Aggiungo il nuovo thread alla lista con tutti i thread creati sino ad ora
    created_threads.append( new_thread )


###################
### LETTURA XML ###
###################

# Leggo il file XML generato dal backend ever_xml
tree = etree.parse(ocarina_ros_path + xml_filename)

# Ottengo la root del system preso in considerazione
root = tree.getroot()

# Ricerco tutti i processi all'interno del system
processes = root.findall("./" +
                            tags['TAG_SUBCOMPONENTS'] + "/" +
                                tags['TAG_SUBCOMPONENT'] + "/" +
                                    "[" + tags['TAG_CATEGORY'] + "='process']")

# Scorro ogni processo. Per ogni processo controllo i subcomponent: in base alle varie tipologie
# di subcomponent avvio la generazione di diversi nodi ROS
for process in processes:
    threads = process.findall("./" +
                                tags['TAG_SUBCOMPONENTS'] + "/" +
                                    tags['TAG_SUBCOMPONENT'] + "/" +
                                        "[" + tags['TAG_CATEGORY'] + "='thread']")

    for thread in threads:
        name = (thread.find( tags['TAG_NAME'] ).text).lower()

        creaNuovoThread( process,
                         thread,
                         AADLThreadMapping.NAME_TO_CLASS.get(name, "Generic") )