import XMLTags
from lxml import etree

def getMainThread(process):
    # Cerco inoltre il MAIN_THREAD che deve essere presente
    main_thread = process.find("./" +
                                XMLTags.tags['TAG_SUBCOMPONENTS'] + "/" +
                                    XMLTags.tags['TAG_SUBCOMPONENT'] + "/" +
                                        "[" + XMLTags.tags['TAG_CATEGORY']  + "='thread']"       + "/" +
                                        "[" + XMLTags.tags['TAG_NAME']      + "='main_thread']" + "/" +
                                        "[" + XMLTags.tags['TAG_NAMESPACE'] + "='core']")

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