import XMLTags
from lxml import etree

"""
    COMPARISON
"""

def areThreadsEqual(thread1, thread2):
    dict_thread1 = thread1.getDescriptionForComparison()
    dict_thread2 = thread2.getDescriptionForComparison()

    if len(dict_thread1) != len(dict_thread2):
        return False

    for key, value in dict_thread1.items():
        if (key not in dict_thread2):
            return False
        if dict_thread2[key] != value:
            return False

    return True


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
        return None

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
        return (None, None);