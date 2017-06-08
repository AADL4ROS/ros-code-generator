# In input ho una tupla del tipo (datatype, datatype_namespace): a partire da questa devo
# fare la traduzione in tipi supportati da ROS
def getROSDatatypeFromAADLDatatype(aadl_datatype):
    namespace   = aadl_datatype[0]
    datatype    = aadl_datatype[1]

    #@TODO: Gestione dei Datatype

    return ("std_msgs", "String", "std_msgs/String.h")
