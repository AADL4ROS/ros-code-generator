from datatypes.Type import String, StdMsgString

# In input ho una tupla del tipo (datatype, datatype_namespace): a partire da questa devo
# fare la traduzione in tipi supportati da ROS
def getROSDatatypeFromAADLDatatype(_aadl_datatype, _associated_class):
    #namespace   = aadl_datatype[0]
    #datatype    = aadl_datatype[1]

    #@TODO: Gestione dei Datatype

    return StdMsgString( _associated_class )
