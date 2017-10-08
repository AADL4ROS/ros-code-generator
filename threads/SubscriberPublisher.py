import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread
from threads.AADLThread import AADLThreadType
from lxml import etree

import threads.AADLThreadFunctionsSupport as tfs

import datatypes.DatatypeConversion as dt

from datatypes.Type import Int, Double, Void, ROS_Subscriber, ROS_Publisher

from variables.Variable import Variable
from methods.Method import Method
from comments.Comment import Comment

class SubscriberPublisher(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, AADLThreadType.SUBSCRIBER_PUBLISHER, _associated_class)
        log.info("SubscriberPublisher thread {}".format(self.name))

        # Parametri del SubscriberPubslisher
        self.main_thread        = None
        self.input_port_name    = "msg_in"
        self.output_port_name   = "msg_out"

        # Parametri comuni
        self.source_text = None

        # Parametri della parte Subcriber
        self.sub_process_port       = None
        self.sub_asn1_source_file   = None
        self.sub_topic              = None
        self.sub_queue_size         = None
        self.sub_callback           = None
        self.input_type             = None

        # Parametri della parte Publisher
        self.pub_process_port       = None
        self.pub_topic              = None
        self.pub_asn1_source_file   = None
        self.output_type            = None

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

        #############
        ### ASN.1 ###
        #############
        #self.sub_asn1_source_file = tfs.getSourceText(self.sub_process_port)

        #if self.sub_asn1_source_file == None:
            # return (False, "Unable to find the ASN.1 file specification.")
        #    log.warning("Unable to find the Subscriber ASN.1 file specification.")

        #log.info("ASN.1 subscriber file: {}".format(self.sub_asn1_source_file))

        ##################
        ### INPUT TYPE ###
        ##################

        (aadl_namespace, aadl_type) = tfs.getPortDatatypeByPort(self.sub_process_port)
        if aadl_namespace == None or aadl_type == None:
            return (False, "Unable to identify process port type")

        raw_output_type = dt.getROSDatatypeFromAADL(aadl_namespace, aadl_type, self.associated_class)

        if raw_output_type == None:
            return (False, "Datatype {} NOT supported".format(raw_output_type))
        else:
            self.input_type = raw_output_type

        #self.input_type = dt.getROSDatatypeFromASN1(self.sub_asn1_source_file, self.associated_class)

        self.input_type.setConst(_const=True)
        self.input_type.setAfterTypeName("::ConstPtr&")

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

        #############
        ### ASN.1 ###
        #############
        #self.pub_asn1_source_file = tfs.getSourceText(self.pub_process_port)

        #if self.pub_asn1_source_file == None:
            # return (False, "Unable to find the ASN.1 file specification.")
        #    log.warning("Unable to find the ASN.1 file specification.")

        # @TODO: leggere il file ASN.1 ed utilizzarlo per la porta
        #log.info("ASN.1 file: {}".format(self.pub_asn1_source_file))

        ###################
        ### OUTPUT TYPE ###
        ###################
        #self.output_type = dt.getROSDatatypeFromASN1(self.pub_asn1_source_file, self.associated_class)

        (aadl_namespace, aadl_type) = tfs.getPortDatatypeByPort(self.pub_process_port)
        if aadl_namespace == None or aadl_type == None:
            return (False, "Unable to identify process port type")

        raw_output_type = dt.getROSDatatypeFromAADL(aadl_namespace, aadl_type, self.associated_class)

        if raw_output_type == None:
            return (False, "Datatype {} NOT supported".format(raw_output_type))
        else:
            self.output_type = raw_output_type

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

        self.main_thread.prepare.addMiddleCode("{} = handle.advertise < {} > (\"{}\", 10);"
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

        self.source_text = tfs.getSourceText(thread_function)

        if self.source_text == None:
            return (False, "Unable to find property Source_Text")

        comment_source_code = Comment(self.associated_class)
        comment_source_code.setComment("Source text: {}".format(self.source_text))
        self.sub_callback.addMiddleCode(comment_source_code)

        return (True, "")