import datatypes.Type

# getROSDatatypeFromAADL
# La funzione traduce un datatype definito in AADL in un datatype
# compatibile con ROS.
def getROSDatatypeFromAADL(aadl_namespace, aadl_type, associated_class):

    generic_type = datatypes.Type.Type(associated_class)

    generic_type.setNamespace(aadl_namespace)

    # RIMUOVO LO 0 (zero) DALLA FINE DELLA STRINGA
    type_name = aadl_type
    if type_name.endswith("0"):
        type_name = type_name[:-1]

    generic_type.setTypeName(type_name)

    return generic_type


def getROSDatatypeFromASN1(asn_type, associated_class, is_msg_or_service = False):

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
        "INTEGER"   : "Int",

        ###################
        ### REAL/DOUBLE ###
        ###################
        "REAL": "Double",

        ###############
        ### BOOLEAN ###
        ###############
        "BOOLEAN": "Bool",

        ############
        ### TIME ###
        ############
        "UTCTIME" : "Ros_Time"
    }

    if asn_type.upper() in mapping_asn_ros:
        type_class_name = mapping_asn_ros[asn_type.upper()]

        # Nel caso in cui si stesse mandando una stringa in un messaggio oppure
        # in un servizio, questa viene gestita come std_msgs::String e non come
        # una classica std::string. Stessa cosa per altre casistiche
        if is_msg_or_service:
            if type_class_name == "String":
                type_class_name = "MsgSrv_String"

            if type_class_name == "Ros_Time":
                type_class_name = "MsgSrv_Time"

            if type_class_name == "Double":
                type_class_name = "Float64"

            if type_class_name == "Int":
                type_class_name = "Int64"

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