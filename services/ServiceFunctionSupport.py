import logging
log = logging.getLogger("root")

import os
from asn1tools.parser import parse_file
from services.Service import Service
from variables.Variable import Variable
from datatypes.DatatypeConversion import getROSDatatypeFromASN1

import global_filepath

asn_default_path = global_filepath.aadl_model_dir

"""
Struttura base per un servizio

Custom_Service DEFINITIONS ::= BEGIN

	Request ::= SEQUENCE {
		a Integer,
		b PrintableString,
		proc BOOLEAN
	}

	Response ::= SEQUENCE {
		appended INTEGER,
		length REAL,
		pose geometry_msgs/Pose
	}

END
"""

def getServiceFromASN1(aadl_namespace, aadl_type, asn_source, associated_class):
    file_path = os.path.join(asn_default_path, asn_source)

    try:
        parsed = parse_file(file_path)
    except:
        log.error("Unable to parse ASN.1 file {}".format(file_path))
        return None

    # Il nome del servizio è la prima ed unica chiave del dizionario del file parsato
    new_service_intestazione = list(parsed.keys())[0]

    service = Service(aadl_namespace, aadl_type)

    requests_asn    = parsed[new_service_intestazione]['types']['Request']['members']
    responses_asn   = parsed[new_service_intestazione]['types']['Response']['members']

    for r in requests_asn:
        var = Variable()
        var.setName( r['name'] )
        var.setType( getROSDatatypeFromASN1(r['type'], associated_class, is_msg_or_service = True) )
        var.setIsParameter()

        service.addRequest(var)

    for r in responses_asn:
        var = Variable()
        var.setName( r['name'] )
        var.setType( getROSDatatypeFromASN1(r['type'], associated_class, is_msg_or_service = True) )
        var.setIsParameter()

        service.addResponse(var)

    # Aggiungo il messaggio al suo relativo system
    service.system.addService(service)

    return service