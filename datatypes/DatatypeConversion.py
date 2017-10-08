import datatypes.Type
import importlib

# getROSDatatypeFromAADL
# La funzione traduce un datatype definito in AADL in un datatype
# compatibile con ROS. Se il datatype definito NON è fra quelli
# standard e mappati, allora un messaggio custom verrà generato
def getROSDatatypeFromAADL(aadl_namespace, aadl_type, associated_class):

    mapping_aadl_to_ros = {
        ################
        ### STD_MSGS ###
        ################
        "std_msgs" : "Std_Msgs",

        #####################
        ### GEOMETRY_MSGS ###
        #####################
        "geometry_msgs": "Geometry_Msgs"
    }

    if  aadl_namespace.lower() in mapping_aadl_to_ros:
        type_class_name = mapping_aadl_to_ros[ aadl_namespace.lower() ]
        type_module = importlib.import_module("datatypes." + type_class_name)

        try:
            type_class = getattr(type_module, aadl_type)
        except AttributeError:
            # @TODO: il tipo del messaggio passato NON fa parte della libreria richiesta.
            # Ad esempio, mentre String fa parte di Std_Msgs, TipoStrano non ne fa parte.
            # Vuol dire che c'è un typo molto probabilmente
            # @TODO: corretto dire che c'è un typo?
            return None

        # Se tutto è OK, ritorno la classe che gestisce il tipo scelto
        return type_class( associated_class )

    else:
        # Si è scelto un messaggio molto probabilmente di tipo custom, quindi
        # lo devo andare a generare
        return None

    return None


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
