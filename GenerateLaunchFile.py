import os
import log
logger = log.setup_custom_logger("root")

# Threads
from launchfile.LaunchFile import LaunchFile
from launchfile.Node import Node
from launchfile.Remap import Remap

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

def saveLaunchFile(launch_file):
    filename = "{}.launch".format(launch_file.system_name)
    source_output_path = os.path.join(output_folder, filename)

    with open(source_output_path, 'w+') as file:
        file.write(launch_file.generateCode())


def getConnectionInfo(c):
    conn_info = c.find("./" + XMLTags.tags['TAG_CONNECTION_PORT_INFO'])

    source_port = conn_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_SOURCE']).text
    source_type = conn_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_SOURCE']).text
    source_name = conn_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_SOURCE_NAME']).text

    dest_port = conn_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_DEST']).text
    dest_type = conn_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_DEST']).text
    dest_name = conn_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_DEST_NAME']).text

    return (source_port, source_type, source_name,
            dest_port, dest_type, dest_name)

def getSubcomponentByName(system_root, process_name):
    try:
        subcomponent = system_root.find("./" +
                                        XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                            XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                                "[" + XMLTags.tags['TAG_NAME'] + "='" + process_name + "']")
        sub_category = subcomponent.find(XMLTags.tags['TAG_CATEGORY']).text
        return (sub_category, subcomponent)
    except Exception:
        return (None, None)

def getInvolvedProcessNamePerConnection(system_root, c):
    try:
        (source_port, source_type, source_name,
         dest_port, dest_type, dest_name) = getConnectionInfo(c)
    except Exception:
        logger.error("Connection between {} and {} is broken:\n" \
                     "\tUnable to read connections".format(source_name, dest_name))
        return None

    nodo_source = {
        'name': source_name,
        'port': source_port
    }
    nodo_dest = {
        'name': dest_name,
        'port': dest_port
    }

    return [nodo_source, nodo_dest]


def generateLaunchFileForSystem(system_root):
    logger.info("Generate Launch File For System {}".format( tfs.getType( system_root ) ))

    launchFile = LaunchFile(system_root)

    # Cerco tutte le connessioni
    system_connections = system_root.findall("./" + XMLTags.tags['TAG_CONNECTIONS'] + "/" +
                                       XMLTags.tags['TAG_CONNECTION'])

    # Genero l'entry del launch file solamente per i nodi che sono connessi a qualche cosa, altrimenti
    # sono dei nodi abbadonati a loro stessi che non devono essere avviati
    for c in system_connections:
        # Cerco le info <port_info> per ciascuna connessione, in modo
        # da sapere quali sono i processi coinvolti
        process_name_list = getInvolvedProcessNamePerConnection(system_root, c)
        if process_name_list is None:
            logger.error("Unable to retrieve involved process in connection: {}"
                         .format(c.find(XMLTags.tags['TAG_NAME']).text))
            return False

        for p in process_name_list:
            (sub_category, subcomponent) = getSubcomponentByName(system_root, p['name'] )

            # Se non trovo nulla salto
            if sub_category == None or subcomponent == None:
                continue

            if sub_category.lower() == "process":
                n = Node(subcomponent)
                status = True
                hasNode = launchFile.hasNode(n)
                if hasNode:
                    n = launchFile.retrieveNode(n.name)

                status = status and n.addPortForTopicFromPortName( p['port'], c )
                if not status:
                    logger.error("Node {} error in remapping topic name"
                                 .format(p['name']))

                if status:
                    if not hasNode:
                        launchFile.addNode(n)
                        logger.info("Node {} added successfully"
                                    .format(p['name']))
                    else:
                        logger.info("Node {} updated successfully"
                                    .format(p['name']))
                else:
                    logger.error("Node {} encountered an error and it was not added to the launch file"
                                 .format(p['name']))

    return launchFile


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
    'system' : system_root,
    'parent' : None
}]

# Mi salvo tutto l'albero dei launch file in modo da poterli includere
# uno dentro l'altro
launch_files = []

while len(systems) > 0:
    system_struct = systems.pop(0)

    s       = system_struct['system']
    parent  = system_struct['parent']

    launch_file = generateLaunchFileForSystem(s)

    launch_files.append(launch_file)

    # Se ho un genitore, ovvero un system che mi ha incluso,
    # lo avviso che dovrà includermi
    if parent != None:
        parent.addSubSystem( launch_files[-1] )
        pass

    # Mi cerco eventuali system dentro ad altri system. Questo serve nel caso in cui un
    # system sia usato come subcomponents di un altro system. La cosa è ricorsiva, poiché
    # di volta in volta la system_root diventa il system considerato.
    # La prima visita si fa comunque alla system root, dopo si passa a visitare ricorsivamente
    # tutti i vari system
    sub_systems = s.findall("./" +
                          XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                          XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                            XMLTags.tags['TAG_SYSTEM'] + "/" +
                          "[" + XMLTags.tags['TAG_CATEGORY'] + "='system']")

    for sub_sys in sub_systems:
        systems.append( {
            'system' : sub_sys,
            'parent' : launch_files[-1]
        } )

# Una volta temrminata l'esplorazione di tutti i system, vado a generare i launch file
for f in launch_files:
    saveLaunchFile(f)