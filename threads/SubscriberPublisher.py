import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs
import messages.MessageFunctionSupport as mfs

import datatypes.DatatypeConversion as dt

from datatypes.Type import Void, ROS_Subscriber, ROS_Publisher

from variables.Variable import Variable
from methods.Method import Method
from comments.Comment import Comment
from datatypes.Type import Type
from libraries.Library import Library

class SubscriberPublisher(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
        log.info("SubscriberPublisher thread {}".format(self.name))

        # Parametri del SubscriberPubslisher
        self.main_thread        = None
        self.input_port_name    = "msg_in"
        self.output_port_name   = "msg_out"

        # Parametri comuni
        self.source_text_file = None

        # Parametri della parte Subcriber
        self.sub_process_port       = None
        self.sub_asn1_source_file   = None
        self.sub_topic              = None
        self.sub_queue_size         = None
        self.sub_callback           = None
        self.input_type             = None
        self.sub_custom_msg         = None

        # Parametri della parte Publisher
        self.pub_process_port       = None
        self.pub_topic              = None
        self.pub_asn1_source_file   = None
        self.output_type            = None
        self.pub_custom_msg         = None

    def populateSubscriberData(self):
        ##################
        ### Input Port ###
        ##################

        # Ottengo la connesione che mappa la porta di input del thread subscriber
        # con quella che entra nel process
        process_input_port = tfs.getConnectionPortInfoByDest(self.process, self.type, self.input_port_name)
        if process_input_port == None:
            return (False, "Unable to find the right binding between process input port and thread input port")

        (source_parent_name, source_name) = tfs.getSourceFromPortInfo(process_input_port)

        if source_parent_name == None or source_name == None:
            return (False, "Unable to find the process input port name")

        self.sub_process_port = tfs.getFeatureByName(self.process, name=source_name)

        if self.sub_process_port == None:
            return (False, "Unable to find the process input port name feature")


        ##################
        ### INPUT TYPE ###
        ##################

        (aadl_namespace, aadl_type) = tfs.getPortDatatypeByPort(self.sub_process_port)
        if aadl_namespace == None or aadl_type == None:
            return (False, "Unable to identify process port type")

        # Controllo se c'è un file ASN.1 associato alla porta. Se c'è allora il tipo di messaggio
        # è custom e lo dovrò generare, mentre se non c'è allora è un messaggio standard ROS
        port_data_info = tfs.getPortDataInfo(self.sub_process_port)
        if port_data_info == None:
            return (False, "Unable to get the port data info for process port")

        port_data_source_asn = tfs.getSourceText(port_data_info)
        if port_data_source_asn == None:
            # Se è None allora non c'è alcun file ASN.1 associato e quindi è un messaggio standard ROS
            raw_output_type = dt.getROSDatatypeFromAADL(aadl_namespace, aadl_type, self.associated_class)
            if raw_output_type == None:
                return (False, "Datatype {} NOT supported".format(raw_output_type))
            else:
                self.input_type = raw_output_type
        else:
            self.custom_message = mfs.getMessageFromASN1(aadl_namespace,
                                                         aadl_type,
                                                         port_data_source_asn,
                                                         self.associated_class)

            self.input_type = Type(self.associated_class)
            self.input_type.setTypeName(self.custom_message.name)
            self.input_type.setNamespace(self.custom_message.namespace)

        self.input_type.setConst(_const=True)
        self.input_type.setAfterTypeName("::ConstPtr&")

        # Associo la librerie del messaggio al tipo di output, sia custom che standard
        input_type_library = Library()
        input_type_library.setPath("{}/{}.h".format(self.input_type.namespace, self.input_type.type_name))

        self.input_type.setLibrary(input_type_library)

        ########################
        ### SUBSCRIBER TOPIC ###
        ########################

        (status, desc) = self.getDefaultTopicName(self.input_port_name, input=True)
        if status == False:
            return (status, desc)

        ##################
        ### QUEUE SIZE ###
        ##################

        queue_size_default_value = 1
        self.queue_size = tfs.getSubscriberQueueSize(self.thread, port_name=self.input_port_name)

        if self.queue_size == None:
            self.queue_size = queue_size_default_value
            log.info("Queue size set to default value: {}".format(self.queue_size))

        ######################
        ### SUBSCRIBER VAR ###
        ######################

        var_subscriber_pub = Variable(self.associated_class)
        var_subscriber_pub.setName("sub_{}".format(self.name))
        var_subscriber_pub.setType(ROS_Subscriber(self.associated_class))

        self.associated_class.addInternalVariable(var_subscriber_pub)

        ###########################
        ### SUBSCRIBER CALLBACK ###
        ###########################

        self.sub_callback = Method(self.associated_class)
        self.sub_callback.method_name = "{}_callback".format(self.name)
        self.sub_callback.return_type = Void(self.associated_class)
        self.sub_callback.namespace = self.associated_class.class_name

        input_var = Variable(self.associated_class)
        input_var.setType(self.input_type)
        input_var.setName("msg")
        input_var.setIsParameter()

        self.sub_callback.addInputParameter(input_var)

        self.sub_callback.addMiddleCode("ROS_INFO(\"%s\", {}->data.c_str());".format(input_var.name))

        self.associated_class.addPrivateMethod(self.sub_callback)

        self.main_thread.prepare.addMiddleCode("{} = handle.subscribe(\"{}\", {}, {}, this);"
                                          .format(var_subscriber_pub.name, self.topic, self.queue_size,
                                                  self.sub_callback.getThreadPointer()))

        return (True, "")

    def populatePublisherData(self):
        ###################
        ### Output Port ###
        ###################

        # Ottengo la connesione che mappa la porta di input del thread subscriber
        # con quella che entra nel process
        process_output_port = tfs.getConnectionPortInfoBySource(self.process, self.type, self.output_port_name)
        if process_output_port == None:
            return (False, "Unable to find the right binding between process input port and thread input port")

        (dest_parent_name, dest_name) = tfs.getDestFromPortInfo(process_output_port)

        if dest_parent_name == None or dest_name == None:
            return (False, "Unable to find the process input port name")

        self.pub_process_port = tfs.getFeatureByName(self.process, name=dest_name)
        if self.pub_process_port == None:
            return (False, "Unable to find the process input port name feature")

        ###################
        ### OUTPUT TYPE ###
        ###################

        (aadl_namespace, aadl_type) = tfs.getPortDatatypeByPort(self.pub_process_port)
        if aadl_namespace == None or aadl_type == None:
            return (False, "Unable to identify process port type")

        # Controllo se c'è un file ASN.1 associato alla porta. Se c'è allora il tipo di messaggio
        # è custom e lo dovrò generare, mentre se non c'è allora è un messaggio standard ROS
        port_data_info = tfs.getPortDataInfo(self.pub_process_port)
        if port_data_info == None:
            return (False, "Unable to get the port data info for process port")

        port_data_source_asn = tfs.getSourceText(port_data_info)
        if port_data_source_asn == None:
            # Se è None allora non c'è alcun file ASN.1 associato e quindi è un messaggio standard ROS
            raw_output_type = dt.getROSDatatypeFromAADL(aadl_namespace, aadl_type, self.associated_class)
            if raw_output_type == None:
                return (False, "Datatype {} NOT supported".format(raw_output_type))
            else:
                self.output_type = raw_output_type
        else:
            self.custom_message = mfs.getMessageFromASN1(aadl_namespace,
                                                         aadl_type,
                                                         port_data_source_asn,
                                                         self.associated_class)

            self.output_type = Type(self.associated_class)
            self.output_type.setTypeName(self.custom_message.name)
            self.output_type.setNamespace(self.custom_message.namespace)

        # Associo la librerie del messaggio al tipo di output, sia custom che standard
        output_type_library = Library()
        output_type_library.setPath("{}/{}.h".format(self.output_type.namespace, self.output_type.type_name))

        self.output_type.setLibrary(output_type_library)

        #############
        ### TOPIC ###
        #############

        (status, desc) = self.getDefaultTopicName( self.output_port_name, output=True )
        if status == False:
            return (status, desc)

        #####################
        ### PUBLISHER VAR ###
        #####################

        var_publisher_pub = Variable(self.associated_class)
        var_publisher_pub.setName("pub_{}".format(self.name))
        var_publisher_pub.setType(ROS_Publisher(self.associated_class))
        self.associated_class.addInternalVariable(var_publisher_pub)

        ######################
        ### PUBLISHER CODE ###
        ######################
        # @TODO: nome variabile del publisher
        self.sub_callback.addBottomCode( "{}.publish(msg);".format(var_publisher_pub.name) )

        self.main_thread.prepare.addMiddleCode("{} = handle.advertise<{}>(\"{}\", 10);"
                                          .format(var_publisher_pub.name, self.output_type.generateCode(), self.topic))

        return (True, "")


    def populateData(self):
        self.main_thread = self.associated_class.getMainThread()

        if self.main_thread == None:
            return (False, "Unable to get the Main Thread")

        thread_function = tfs.getSubprogram( self.thread )
        if thread_function == None:
            return (False, "Unable to find the right Subprogram")

        ##################
        ### SUBSCRIBER ###
        ##################
        (status, desc) = self.populateSubscriberData()
        if status == False:
            return (status, desc)

        #################
        ### PUBLISHER ###
        #################
        (status, desc) = self.populatePublisherData()
        if status == False:
            return (status, desc)

        ###################
        ### SOURCE TEXT ###
        ###################

        self.source_text_file = self.createSourceTextFileFromSourceText(tfs.getSourceText(thread_function),
                                                                                tfs.getSourceName(thread_function))
        # Aggiungo la chiamata alla funzione custome
        if self.source_text_file != None:
            code = "{};".format(self.source_text_file.generateInlineCode())
            self.sub_callback.addMiddleCode(code)
        else:
            return (False, "Unable to find Source_Text or Source_Name")

        return (True, "")