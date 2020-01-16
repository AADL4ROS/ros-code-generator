import logging
import os
import json
from services.Service import Service
from variables.Variable import Variable
from datatypes.DatatypeConversion import getROSMsgtypeFromJSON


import global_filepath

log = logging.getLogger("root")
asn_default_path = global_filepath.aadl_model_dir


def getServiceFromJSON(aadl_namespace, aadl_type, json_source, associated_class):
    file_path = os.path.join(asn_default_path, json_source)

    try:
        parsed = json.load(open(file_path))
    except json.JSONDecodeError:
        log.error("Unable to load JSON file {}".format(file_path))
        return None

    service = Service(aadl_namespace, aadl_type)

    requests_json = parsed['Request']['properties']
    responses_json = parsed['Response']['properties']

    for key, value in requests_json.items():
        var = Variable()
        var.setName(key)
        var.setType(getROSMsgtypeFromJSON(value['type'], associated_class))
        var.setIsParameter()
        service.addRequest(var)

    for key, value in responses_json.items():
        var = Variable()
        var.setName(key)
        # TODO string format + nested messages
        var.setType(getROSMsgtypeFromJSON(value['type'], associated_class))
        var.setIsParameter()
        service.addResponse(var)

    # Aggiungo il messaggio al suo relativo system
    service.system.addService(service)

    return service
