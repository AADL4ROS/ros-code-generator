import datatypes.Type
import libraries.Library


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


def getROSMsgtypeFromJSON(json_type, associated_class, str_format=None):
    mapping_json_ros = {
        "string": "MsgSrv_String",
        "number": "Float64",
        "integer": "Int64",
        "boolean": "Bool"
    }

    if json_type.lower() in mapping_json_ros:
        type_class_name = mapping_json_ros[json_type.lower()]

        # ugly special case
        if type_class_name == "String" and str_format == "date-time":
            type_class_name = "Ros_Time"

        type_class = getattr(datatypes.Type, type_class_name)

    return type_class(associated_class)


def manageComplexType(complex_type, include, associated_class):
    generic_type = datatypes.Type.Type(associated_class)
    generic_type.setTypeName(complex_type)
    lib = libraries.Library.Library(associated_class)
    lib.setPath(include)
    generic_type.setLibrary(lib)

    return generic_type


def getROSDatatypeFromJSON(json_type, associated_class):
    mapping_json_ros = {
        "string": "String",
        "number": "Double",
        "integer": "Int",
        "boolean": "Bool"
    }

    if json_type.lower() in mapping_json_ros:
        type_class_name = mapping_json_ros[json_type.lower()]
        type_class = getattr(datatypes.Type, type_class_name)

        return type_class(associated_class)
