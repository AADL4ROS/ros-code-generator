import logging
log = logging.getLogger("root")

import os
from asn1tools.parser import parse_file
from messages.Message import Message
from variables.Variable import Variable
from datatypes.DatatypeConversion import getROSDatatypeFromASN1

asn_default_path = "../ocarina-ros/"

def getMessageFromASN1(asn_source, associated_class):
    file_path = os.path.join(asn_default_path, asn_source)

    try:
        parsed = parse_file(file_path)
    except:
        log.error("Unable to parse ASN.1 file {}".format(file_path))
        return None

    # Il nome del messaggio è la prima ed unica chiave del dizionario del file parsato
    new_message_name = list(parsed.keys())[0]

    message = Message(new_message_name)

    parameters_asn = parsed[new_message_name]['values']

    for val, spec in parameters_asn.items():
        var = Variable()
        var.setName( val )
        var.setType( getROSDatatypeFromASN1(spec['value'], associated_class, is_msg_or_service = True) )
        var.setIsParameter()

        message.addParameter(var)

    return message