import importlib
import log
logger = log.setup_custom_logger("root")

# Threads
from threads.AADLThread import AADLThreadMapping
from threads.AADLThread import AADLThreadType

from threads.AADLThreadFunctionsSupport import areThreadsEqual

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

# Salvo il momento della generazione automatica, in modo da poterlo segnare nel file
today           = datetime.datetime.now()
generated_on    = today.strftime("%d/%m/%Y %H:%M:%S")
print( "Avvio generazione: {}".format(generated_on) )

created_threads = []

def creaNuovoThread( process, thread, classname ):
    # Importo il modulo che contiene la tipologia di thread che voglio aggiungere
    thread_module   = importlib.import_module("threads." + classname)

    # Ottengo la classe che gestisce quel particolare thread
    thread_class    = getattr(thread_module, classname)

    # Creo una nuova istanza della classe relativa al thread e lancio quindi la procedura di
    # creazione effettiva del codice
    new_thread      = thread_class(process, thread)

    (status, error_desc) = new_thread.generateCode()
    if not status:
        # Si è verificato un errore, per il momento lo scrivo a schermo
        logger.error("Errore durante la generazione del codice")
        logger.error(error_desc)
    else:
        # Se tutto è andato a buon fine proseguo

        # Il nuovo thread viene generato e salvato solamente se non ne esiste già uno identico,
        # in tal caso significherebbe avere due nodi identici lanciati due volte che non hanno
        # bisogno di codice separato (basti pensare a due sensori uguali ad esempio).
        save_new_thread = True
        for t in created_threads:
            if areThreadsEqual(new_thread, t):
                save_new_thread = False
                break

        if save_new_thread:
            new_thread.saveOutputSource()
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

    # Se ho più di un thread significa che uno dei due è sicuramente un main thread mentre
    # l'altro specifica meglio il nodo. Avere solo un thread vuol dire che ho solamente
    # il main thread

    for thread in threads:
        name = (thread.find( XMLTags.tags['TAG_NAME'] ).text).lower()

        if name != AADLThreadType.MAIN_THREAD:
            creaNuovoThread(    process,
                                thread,
                                AADLThreadMapping.NAME_TO_CLASS.get(name, "Generic") )