import XMLTags
from lxml import etree

"""
    MAIN THREAD
"""

def getMainThread(process):
    # Cerco inoltre il MAIN_THREAD che deve essere presente
    main_thread = process.find("./" +
                                XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                    XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                        "[" + XMLTags.tags['TAG_CATEGORY']  + "='thread']"       + "/" +
                                        "[" + XMLTags.tags['TAG_NAME']      + "='main_thread']" + "/" +
                                        "[" + XMLTags.tags['TAG_NAMESPACE'] + "='ros']")

    return main_thread

# Ritorna una tupla del tipo (prepare, tear_down, error_handler)
def getMainThreadFunctions(main_thread):
    # Prepare
    prepare = main_thread.find("./" +
                                  XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                  XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                  "[" + XMLTags.tags['TAG_CATEGORY']    + "='subprogram']" +
                                  "[" + XMLTags.tags['TAG_NAME']        + "='prepare']")

    # TearDown
    tear_down = main_thread.find("./" +
                               XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                               XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                               "[" + XMLTags.tags['TAG_CATEGORY']   + "='subprogram']" +
                               "[" + XMLTags.tags['TAG_NAME']       + "='tear_down']")

    # Error Handler
    error_handler = main_thread.find("./" +
                                XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                "[" + XMLTags.tags['TAG_CATEGORY']  + "='subprogram']" +
                                "[" + XMLTags.tags['TAG_NAME']      + "='error_handler']")

    return (prepare, tear_down, error_handler)

def getSubprogram(thread):
    thread_function = thread.find("./" +
                                  XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                  XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                       "[" + XMLTags.tags['TAG_CATEGORY'] + "='subprogram']")

    return thread_function

"""
    FEATURES
"""

####################
### Port by Name ###
####################

def getFeatureByName(start, name):
    try:
        feature_port_by_name = start.find("./" +
                                            XMLTags.tags['TAG_FEATURES'] + "/" +
                                                XMLTags.tags['TAG_FEATURE'] + "/" +
                                                    "[" + XMLTags.tags['TAG_PROPERTY_NAME'] + "='" + name +  "']")

        return feature_port_by_name
    except AttributeError:
        return None

def getPortNameByPort(port):
    try:
        return port.find(XMLTags.tags['TAG_FEATURE_NAME']).text
    except AttributeError:
        return None

def getPortDatatypeByPort(port):
    try:
        return (port.find(XMLTags.tags['TAG_FEATURE_PORT_DATA_TYPE_NAMESPACE']).text,
                port.find(XMLTags.tags['TAG_FEATURE_PORT_DATA_TYPE']).text)
    except AttributeError:
        return (None, None)

"""
    CONNECTIONS
"""

def getConnectionPortInfoBySource(start, parent_source, name):
    try:
        conn_by_source = start.find("./" +
                                    XMLTags.tags['TAG_CONNECTIONS'] + "/" +
                                    XMLTags.tags['TAG_CONNECTION'] + "/" +
                                    XMLTags.tags['TAG_CONNECTION_PORT_INFO'] + "/" +
                                    "[" + XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_SOURCE'] + "='" + parent_source +  "']" +
                                    "[" + XMLTags.tags['TAG_CONNECTION_PORT_INFO_SOURCE'] + "='" + name + "']")

        return conn_by_source
    except AttributeError:
        return None

def getConnectionPortInfoByDest(start, parent_dest, name):
    try:
        conn_by_dest = start.find("./" +
                                    XMLTags.tags['TAG_CONNECTIONS'] + "/" +
                                    XMLTags.tags['TAG_CONNECTION'] + "/" +
                                    XMLTags.tags['TAG_CONNECTION_PORT_INFO'] + "/" +
                                    "[" + XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_DEST'] + "='" + parent_dest +  "']" +
                                    "[" + XMLTags.tags['TAG_CONNECTION_PORT_INFO_DEST'] + "='" + name + "']")

        return conn_by_dest
    except AttributeError:
        return None

def getSourceFromPortInfo(port_info):
    try:
        return (port_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_SOURCE']).text,
                port_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_SOURCE']).text)
    except:
        return (None, None)

def getDestFromPortInfo(port_info):
    try:
        return (port_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_PARENT_DEST']).text,
                port_info.find(XMLTags.tags['TAG_CONNECTION_PORT_INFO_DEST']).text)
    except:
        return (None, None)

"""
    NAMESPACE
"""

def getNamespace(start):
    return start.find(XMLTags.tags['TAG_NAMESPACE']).text

"""
    TYPE
"""

def getType(start):
    return start.find(XMLTags.tags['TAG_TYPE']).text


"""
    NAME
"""

def getName(start):
    return start.find( XMLTags.tags['TAG_NAME'] ).text


"""
    PROPERTIES
"""

###################
### Source Text ###
###################
def getSourceText(start):
    try:
        source_text_property = start.find("./" +
                                            XMLTags.tags['TAG_PROPERTIES'] + "/" +
                                                XMLTags.tags['TAG_PROPERTY'] + "/" +
                                                    "[" + XMLTags.tags['TAG_PROPERTY_NAME'] + "='Source_Text']")

        source_text = source_text_property.find(XMLTags.tags['TAG_PROPERTY_VALUE']).text
        return source_text
    except AttributeError:
        return None

##############
### Period ###
##############
def getPeriod(start):
    try:
        period_property = start.find("./" +
                                     XMLTags.tags['TAG_PROPERTIES'] + "/" +
                                        XMLTags.tags['TAG_PROPERTY'] + "/" +
                                           "[" + XMLTags.tags['TAG_PROPERTY_NAME'] + "='Period']")

        period      = period_property.find(XMLTags.tags['TAG_PROPERTY_VALUE']).text
        period_unit = period_property.find(XMLTags.tags['TAG_PROPERTY_UNIT']).text

        return (period, period_unit)

    except AttributeError:
        return (None, None)

#############
### TOPIC ###
#############

TOPIC_PROPERTIES_NAMESPACE = "topic_properties"

def getTopicName(start):
    try:
        topic_property = start.find("./" + XMLTags.tags['TAG_CONNECTIONS'] + "/" +
                                                XMLTags.tags['TAG_CONNECTION'] + "/" +
                                                    XMLTags.tags['TAG_PROPERTIES'] + "/" +
                                                        XMLTags.tags['TAG_PROPERTY'] + "/" +
                                                            "[" + XMLTags.tags['TAG_PROPERTY_NAME'] + "='Name']" +
                                                                "[" + XMLTags.tags['TAG_PROPERTY_NAMESPACE'] + "='" +
                                                                                    TOPIC_PROPERTIES_NAMESPACE + "']")

        topic = topic_property.find(XMLTags.tags['TAG_PROPERTY_VALUE']).text
        return (TOPIC_PROPERTIES_NAMESPACE, topic)
    except AttributeError:
        return (None, None)

##################
### QUEUE SIZE ###
##################

def getSubscriberQueueSize(start):
    try:
        queue_size_property = start.find("./" + XMLTags.tags['TAG_FEATURES'] + "/" +
                                                XMLTags.tags['TAG_FEATURE'] + "/" +
                                                "[" + XMLTags.tags['TAG_FEATURE_NAME'] + "='msg']" + "/" +
                                                    XMLTags.tags['TAG_PROPERTIES'] + "/" +
                                                        XMLTags.tags['TAG_PROPERTY'] + "/" +
                                                            "[" + XMLTags.tags['TAG_PROPERTY_NAME'] + "='Queue_size']")

        queue_size = queue_size_property.find(XMLTags.tags['TAG_PROPERTY_VALUE']).text
        return queue_size
    except AttributeError:
        return None