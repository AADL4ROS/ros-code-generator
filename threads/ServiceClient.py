import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs

from datatypes.Type import Void, ROS_ServiceClient

from variables.Variable import Variable
import service.ServiceFunctionSupport as sfs
from libraries.Library import Library

class ServiceClient(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
        log.info("Service Client thread {}".format(self.name))

        # E' la porta che fornisce il servizio, in AADL è una provides subprogram access
        self.output_port_name   = "srv"
        self.caller_name        = "caller"

        # Parametri del Subscriber
        self.process_port           = None # La porta requires subprogram access del process
        self.source_text            = None
        self.asn_description        = None
        self.service_name           = None
        self.default_service_name   = None
        self.service                = None

    def populateData(self):
        main_thread = self.associated_class.getMainThread()

        if main_thread == None:
            return (False, "Unable to get the Main Thread")

        ###################
        ### Output Port ###
        ###################

        # Essendo birezeizonale posso trovare la connessione sia come source che come dest
        conn_by_source = True
        process_output_port = tfs.getConnectionPortInfoBySource(self.process, self.type, self.output_port_name)
        if process_output_port == None:
            conn_by_source = False
            process_output_port = tfs.getConnectionPortInfoByDest(self.process, self.type, self.output_port_name)

        if process_output_port == None:
            return (False, "Unable to find the right binding between process requires subprogram access port and "
                            "thread input port")

        if conn_by_source:
            (dest_parent_name, dest_name) = tfs.getDestFromPortInfo(process_output_port)
        else:
            (dest_parent_name, dest_name) = tfs.getSourceFromPortInfo(process_output_port)

        if dest_parent_name == None or dest_name == None:
            return (False, "Unable to find the process requires subprogram access port name")

        self.process_port = tfs.getFeatureByName(self.process, name=dest_name)

        if self.process_port == None:
            return (False, "Unable to find the process requires subprogram access port feature")

        # Dopo aver trovato la porta del process, controllo il nome di default del
        # service associato
        (topic_namespace, self.default_service_name) = tfs.getDefaultTopicName(self.process_port)

        if self.default_service_name == None:
            self.default_service_name = "service_name_default"
            log.warning("Default Service Name not found, set as {} as default.".format(self.default_service_name))

        ##################################
        ### ASN.1 Request and Response ###
        ##################################

        caller = tfs.getSubcomponentByInfo(self.thread,
                                           name         = self.caller_name,
                                           namespace    = "ros",
                                           category     = "subprogram")

        if caller == None:
            return (False, "Unable to find the right subprogram associated with the service")

        self.asn_description = tfs.getSourceText(caller)

        if self.asn_description == None:
            return (False, "Unable to find property Source_Text for the service caller with ASN.1 description")

        # Creo il servizio custom e lo associo al nodo che lo ha generato
        self.service = sfs.getServiceFromASN1( self.asn_description, self.associated_class)
        if self.service == None:
            return (False, "Error in ASN.1 parsing")

        self.associated_class.addService( self.service )

        # Genero ed aggiungo la libreria del service al nodo
        service_library = Library(self.associated_class)
        service_library.setPath( "{}/{}.h".format(self.associated_class.namespace, self.service.name) )
        self.associated_class.addLibrary(service_library)

        ##########################
        ### SERVICE CLIENT VAR ###
        ##########################

        var_serviceclient = Variable(self.associated_class)
        var_serviceclient.setName( "service_client_{}".format(self.name) )
        var_serviceclient.setType( ROS_ServiceClient( self.associated_class) )
        self.associated_class.addInternalVariable( var_serviceclient )

        main_thread.prepare.addMiddleCode("{} = handle.serviceClient<{}::{}>(\"{}\");"
                                          .format(var_serviceclient.name,
                                                  self.associated_class.namespace,
                                                  self.service.name,
                                                  self.default_service_name))

        return (True, "")