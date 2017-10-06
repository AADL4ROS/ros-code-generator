import datatypes.Type

# In input ho una tupla del tipo (datatype, datatype_namespace): a partire da questa devo
# fare la traduzione in tipi supportati da ROS
def getROSDatatypeFromASN1(_asn1_source, _associated_class, _custom_types = {}):
    #@TODO: Gestione dei Datatype

    default_asn_type = "__DEFAULT__"

    mapping_asn_ros = {
        "__DEFAULT__"       : "StdMsgString",
        "REAL"              : "Double",
        "INTEGER"           : "Int",
        "BOOLEAN"           : "Bool",
        "STRING"            : "String",
        "UTF8STRING"        : "String"
    }

    # Se è un tipo custom definito dall'utente (un oggetto), allora
    # io non posso farci nulla, altrimenti preparo tutto quanto
    if _asn1_source in _custom_types:
        return datatypes.Type.Object(_associated_class, _asn1_source)

    else:
        if _asn1_source.upper() in mapping_asn_ros:
            classname = mapping_asn_ros[_asn1_source.upper()]
        else:
            classname = mapping_asn_ros[default_asn_type]

    # Ottengo la classe che gestisce quel particolare thread
    type_class = getattr(datatypes.Type, classname)

    return type_class( _associated_class )
