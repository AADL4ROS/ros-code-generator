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
xml_filename        = "container.tlk_lis_ever_xml.xml"
json_filename       = "tag_ever_xml.json"

#############
### SETUP ###
#############

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

def getASN1FileForPort(system_root, port, type, name):
    feature = system_root.find("./" + XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                       XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                        "[" + XMLTags.tags['TAG_NAME'] + "='" + name + "']" + "/" +
                                        "[" + XMLTags.tags['TAG_TYPE'] + "='" + type + "']" + "/" +
                                            XMLTags.tags['TAG_FEATURES'] + "/" +
                                            XMLTags.tags['TAG_FEATURE'] + "/" +
                                            "[" + XMLTags.tags['TAG_FEATURE_NAME'] + "='" + port + "']" )

    source_text_property = feature.find("./" +
                                      XMLTags.tags['TAG_PROPERTIES'] + "/" +
                                      XMLTags.tags['TAG_PROPERTY'] + "/" +
                                      "[" + XMLTags.tags['TAG_PROPERTY_NAME'] + "='Source_Text']")

    asn1source = source_text_property.find(XMLTags.tags['TAG_PROPERTY_VALUE']).text

    return asn1source

def checkConnection(system_root, c):
    try:
        (source_port, source_type, source_name,
         dest_port, dest_type, dest_name) = getConnectionInfo(c)
    except:
        logger.error("Connection between {} and {} has broken datatype:\n" \
                     "\tUnable to read connections".format(source_name, dest_name))
        return False

    try:
        sourceASN1  = getASN1FileForPort(system_root, source_port, source_type, source_name)
        destASN1    = getASN1FileForPort(system_root, dest_port, dest_type, dest_name)
    except:
        logger.error("Connection between {} and {} has broken datatype:\n" \
                        "\tUnable to read ASN.1 files".format(source_name, dest_name))
        return False

    if sourceASN1 != destASN1:
        logger.error("Connection between {} and {} has broken datatype:\n" \
                     "\tInput datatype: {}\n" \
                     "\tOutput datatype: {}".format(source_name, dest_name, sourceASN1, destASN1))
        return False
    else:
        logger.info("Connection between {} and {} is OK.".format(source_name, dest_name))
        return True


def checkConnectionsForSystem(system_root):
    logger.info("Check Connections For System {}".format( tfs.getType( system_root ) ))

    # Cerco tutte le connessioni
    system_connections = system_root.findall("./" + XMLTags.tags['TAG_CONNECTIONS'] + "/" +
                                       XMLTags.tags['TAG_CONNECTION'])

    for c in system_connections:
        # Cerco le info <port_info> per ciascuna connessione, in modo
        # da sapere qual è la porta in ingresso ed in uscita
        checkConnection(system_root, c)


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

    checkConnectionsForSystem(s)

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
