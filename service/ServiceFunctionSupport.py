import logging
log = logging.getLogger("root")

import os
from asn1tools.parser import parse_file
from service.Service import Service
from variables.Variable import Variable
from datatypes.DatatypeConversion import getROSDatatypeFromASN1

asn_default_path = "../ocarina-ros/"

def getServiceFromASN1(asn_source, associated_class):
    file_path = os.path.join(asn_default_path, asn_source)

    try:
        parsed = parse_file(file_path)
    except:
        log.error("Unable to parse ASN.1 file {}".format(file_path))
        return None

    # Il nome del servizio è la prima ed unica chiave del dizionario del file parsato
    new_service_name = list(parsed.keys())[0]

    service = Service(new_service_name)

    requests_asn    = parsed[new_service_name]['types']['Request']['members']
    responses_asn   = parsed[new_service_name]['types']['Response']['members']

    for r in requests_asn:
        var = Variable()
        var.setName( r['name'] )
        var.setType( getROSDatatypeFromASN1(r['type'], associated_class) )
        var.setIsParameter()

        service.addRequest(var)

    for r in responses_asn:
        var = Variable()
        var.setName( r['name'] )
        var.setType( getROSDatatypeFromASN1(r['type'], associated_class) )
        var.setIsParameter()

        service.addResponse(var)

    return service