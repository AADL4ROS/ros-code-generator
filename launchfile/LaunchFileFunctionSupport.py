import XMLTags

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