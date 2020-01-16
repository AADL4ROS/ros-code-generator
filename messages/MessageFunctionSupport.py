import logging
import os
import json
from messages.Message import Message
from variables.Variable import Variable
from datatypes.DatatypeConversion import getROSMsgtypeFromJSON
import global_filepath

log = logging.getLogger("root")
json_default_path = global_filepath.aadl_model_dir


def getMessageFromJSON(aadl_namespace, aadl_type, json_source, associated_class):
    file_path = os.path.join(json_default_path, json_source)

    try:
        parsed = json.load(open(file_path))
    except json.JSONDecodeError:
        log.error("Unable to load JSON file {}".format(file_path))
        return None

    # new_msg_header = list(parsed.keys())[0]

    message = Message(aadl_namespace, aadl_type)

    # parameters_asn = parsed[new_msg_header]['values']

    for key, value in parsed["properties"].items():
        var = Variable()
        var.setName(key)
        # TODO string format + nested messages
        var.setType(getROSMsgtypeFromJSON(value["type"], associated_class))
        var.setIsParameter()

        message.addParameter(var)

    # Aggiungo il messaggio al suo relativo system
    message.system.addMessage(message)

    return message
