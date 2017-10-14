import datatypes.Type
import importlib
from messages import CustomMessage

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


def getROSDatatypeFromASN1(asn_type, associated_class):

    mapping_asn_ros = {
        ##############
        ### STRING ###
        ##############
        "PRINTABLESTRING"   : "String",
        "STRING"            : "String",
        "NUMERICSTRING"     : "String",
        "IA5STRING"         : "String",

        ###############
        ### INTEGER ###
        ###############
        "INTEGER"   : "Int64",

        ###################
        ### REAL/DOUBLE ###
        ###################
        "REAL": "Double",

        ###############
        ### BOOLEAN ###
        ###############
        "BOOLEAN": "Bool"

    }

    if asn_type.upper() in mapping_asn_ros:
        type_class_name = mapping_asn_ros[asn_type.upper()]
        type_class = getattr(datatypes.Type, type_class_name)

        return type_class(associated_class)
    else:
        generic_type = datatypes.Type.Type(associated_class)

        # Cerco il nome ed il namespace in caso di cose come std_msgs::String
        type_component = asn_type.split("/")

        if len(type_component) == 1:
            generic_type.setTypeName(asn_type)
        elif len(type_component) == 2:
            generic_type.setNamespace(type_component[0])
            generic_type.setTypeName(type_component[1])
        else:
            # Casistica che non dovrebbe mai accadere, ma la gestiamo per sicurezza
            generic_type.setTypeName(asn_type)

        return generic_type