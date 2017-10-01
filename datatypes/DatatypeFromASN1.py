from datatypes.Type import String, StdMsgString

# In input ho una tupla del tipo (datatype, datatype_namespace): a partire da questa devo
# fare la traduzione in tipi supportati da ROS
def getROSDatatypeFromASN1(_asn1_source, _associated_class):
    #@TODO: Gestione dei Datatype

    return StdMsgString( _associated_class )
