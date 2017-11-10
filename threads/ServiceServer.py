import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs

from datatypes.Type import Bool, ROS_ServiceServer, ROS_ServiceServer_Request, ROS_ServiceServer_Response

from comments.Comment import Comment
from variables.Variable import Variable
import services.ServiceFunctionSupport as sfs
from libraries.Library import Library
from methods.Method import Method

class ServiceServer(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
        log.info("Service Server thread {}".format(self.name))

        # E' la porta che fornisce il servizio, in AADL è una provides subprogram access
        self.input_port_name    = "srv"
        self.called_name        = "called"
        self.function_name      = "function"

        # Parametri del Subscriber
        self.process_port           = None # La porta provides subprogram access del process
        self.source_text_function   = None
        self.asn_description        = None
        self.service_name           = None
        self.default_service_name   = None
        self.service                = None

    def populateData(self):
        main_thread = self.associated_class.getMainThread()

        if main_thread == None:
            return (False, "Unable to get the Main Thread")

        ############################
        ### TRANSFORMATION FRAME ###
        ############################

        # Controllo l'uso del Transformation Frame
        self.thread_uses_tf = self.setUsesTransformationFrame()

        ###################
        ### Output Port ###
        ###################

        # Essendo birezeizonale posso trovare la connessione sia come source che come dest
        conn_by_source = True
        process_input_port = tfs.getConnectionPortInfoBySource(self.process, self.type, self.input_port_name)

        if process_input_port == None:
            conn_by_source = False
            process_output_port = tfs.getConnectionPortInfoByDest(self.process, self.type, self.input_port_name)

        if process_input_port == None:
            return (False, "Unable to find the right binding between process requires subprogram access port and "
                           "thread input port")

        if conn_by_source:
            (source_parent_name, source_name) = tfs.getDestFromPortInfo(process_input_port)
        else:
            (source_parent_name, source_name) = tfs.getSourceFromPortInfo(process_output_port)

        if source_parent_name == None or source_name == None:
            return (False, "Unable to find the process provides subprogram access port name")

        self.process_port = tfs.getFeatureByName(self.process, name=source_name)

        if self.process_port == None:
            return (False, "Unable to find the process provides subprogram access port feature")

        # Dopo aver trovato la porta del process, controllo il nome di default del
        # services associato
        (topic_namespace, self.default_service_name) = tfs.getDefaultTopicName(self.process_port)

        if self.default_service_name == None:
            self.default_service_name = "service_name_default"
            log.warning("Default Service Name not found, set as {} as default.".format(self.default_service_name))

        ##################################
        ### ASN.1 Request and Response ###
        ##################################

        (aadl_namespace, aadl_type) = tfs.getPortDatatypeByPort(self.process_port)
        if aadl_namespace == None or aadl_type == None:
            return (False, "Unable to identify process port type")

        # Controllo se c'è un file ASN.1 associato alla porta. Se c'è allora il tipo di servizio
        # è custom e lo dovrò generare, mentre se non c'è allora è un servizio standard ROS
        port_data_info = tfs.getPortDataInfo(self.process_port)
        if port_data_info == None:
            return (False, "Unable to get the port data info for process port")

        self.asn_description = tfs.getSourceText(port_data_info)

        if self.asn_description == None:
            # @TODO: Standard Service
            log.warning("STANDARD SERVICE")
            #return (False, "Unable to find property Source_Text for the services caller with ASN.1 description")
        else:
            # Creo il servizio custom e lo associo al nodo che lo ha generato
            self.service = sfs.getServiceFromASN1(aadl_namespace,
                                                  aadl_type,
                                                  self.asn_description,
                                                  self.associated_class)
            if self.service == None:
                return (False, "Error in ASN.1 parsing")

        # Genero ed aggiungo la libreria del services al nodo
        service_library = Library(self.associated_class)
        service_library.setPath( "{}/{}.h".format(self.service.namespace, self.service.name) )
        self.associated_class.addLibrary(service_library)

        ##########################
        ### SERVICE SERVER VAR ###
        ##########################

        var_service_server = Variable(self.associated_class)
        var_service_server.setName( "service_server_{}".format(self.name) )
        var_service_server.setType( ROS_ServiceServer( self.associated_class) )
        self.associated_class.addInternalVariable( var_service_server )

        ###############################
        ### SERVICE SERVER CALLBACK ###
        ###############################

        self.server_callback = Method(self.associated_class)
        self.server_callback.method_name = "{}_service_callback".format(self.name)
        self.server_callback.return_type = Bool(self.associated_class)
        self.server_callback.namespace = self.associated_class.class_name

        # REQUEST Parameter
        input_param_req = Variable(self.associated_class)
        input_param_req.setName("&req")
        input_param_req.setType(
                ROS_ServiceServer_Request(self.associated_class,
                                          "{}::{}".format(self.service.namespace, self.service.name))
        )
        input_param_req.setIsParameter()

        # RESPONSE Parameter
        input_param_res = Variable(self.associated_class)
        input_param_res.setName("&res")
        input_param_res.setType(
                ROS_ServiceServer_Response(self.associated_class,
                                          "{}::{}".format(self.service.namespace, self.service.name))
        )
        input_param_res.setIsParameter()

        self.server_callback.addInputParameter(input_param_req)
        self.server_callback.addInputParameter(input_param_res)

        ###################
        ### SOURCE TEXT ###
        ###################
        function = tfs.getSubcomponentByInfo(self.thread,
                                           name         = self.function_name,
                                           namespace    = "ros",
                                           category     = "subprogram")
        if function == None:
            return (False, "Unable to find the function subprogram")

        self.source_text_function = self.createSourceTextFileFromSourceText(tfs.getSourceText(function),
                                                                        tfs.getSourceName(function))

        if self.source_text_function == None:
            return (False, "Unable to find property Source_Text or Source_Name")

        self.source_text_function.setTF( self.thread_uses_tf )

        # Aggiungo la chiamata alla funzione custom
        if self.source_text_function != None:
            self.source_text_function.addServiceReqAndRes(input_param_req, input_param_res)
            self.source_text_function.addLibrary(service_library)

            self.source_text_function.setFunctionType( Bool(self.associated_class) )
            code = "return {};".format(self.source_text_function.generateInlineCode())
            self.server_callback.addMiddleCode(code)

        self.associated_class.addPrivateMethod(self.server_callback)

        main_thread.prepare.addMiddleCode("{} = handle.advertiseService(\"{}\", {}, this);"
                                          .format(var_service_server.name,
                                                  self.default_service_name,
                                                  self.server_callback.getThreadPointer()))

        return (True, "")