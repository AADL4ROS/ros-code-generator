import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread
from threads.AADLThread import AADLThreadType
from lxml import etree

import threads.AADLThreadFunctionsSupport as tfs

import datatypes.DatatypeFromPort as dt

from datatypes.Type import Int, Double, Void, ROS_Subscriber

from variables.Variable import Variable
from methods.Method import Method
from comments.Comment import Comment

class Subscriber(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, AADLThreadType.SUBSCRIBER, _associated_class)
        log.info("Subscriber thread {}".format(self.name))

        # Parametri del Subscriber
        self.source_text        = None
        self.topic              = None
        self.subscriberCallback = None
        self.input_type         = None
        self.asn1_source_file   = None

    def populateData(self):
        main_thread = self.associated_class.getMainThread()

        if main_thread == None:
            return (False, "Unable to get the Main Thread")

        # Ottengo le informazioni necessarie per i thread di tipo Subscriber:
        # - Source Text
        # - Topic
        # - Input Type

        thread_function = tfs.getSubprogram( self.thread )
        if thread_function == None:
            return (False, "Unable to find the right Subprogram")

        ##################
        ### Input Port ###
        ##################

        # Ottengo la connesione che mappa la porta di input del thread subscriber
        # con quella che entra nel process
        process_input_port = tfs.getConnectionPortInfoByDest(self.process, self.type, "msg")
        if process_input_port == None:
            return (False, "Unable to find the right binding between process input port and thread input port")

        (source_parent_name, source_name) = tfs.getSourceFromPortInfo(process_input_port)

        if source_parent_name == None or source_name == None:
            return (False, "Unable to find the process input port name")

        process_port = tfs.getFeatureByName(self.process, name=source_name)
        if process_port == None:
            return (False, "Unable to find the process input port name feature")

        self.asn1_source_file = tfs.getSourceText( process_port )

        if self.asn1_source_file == None:
            #return (False, "Unable to find the ASN.1 file specification.")
            log.warning("Unable to find the ASN.1 file specification.")

        # @TODO: leggere il file ASN.1 ed utilizzarlo per la porta
        log.info("ASN.1 file: {}".format(self.asn1_source_file) )

        # Ottengo la porta in output per i thread di tipo Subscriber
        aadl_input_port = tfs.getFeatureByName(self.thread, name = "msg")

        if aadl_input_port == None:
            return (False, "Unable to find the default input port named msg")

        (aadl_input_port_datatype_namespace, aadl_input_port_datatype) = tfs.getPortDatatypeByPort( aadl_input_port )

        self.input_type = dt.getROSDatatypeFromAADLDatatype( (aadl_input_port_datatype_namespace, aadl_input_port_datatype),
                                                                self.associated_class )

        self.input_type.setConst(_const=True)
        self.input_type.setAfterTypeName("::ConstPtr&")

        ###################
        ### Source Text ###
        ###################

        self.source_text = tfs.getSourceText( thread_function )

        if self.source_text == None:
            return (False, "Unable to find property Source_Text")

        #############
        ### TOPIC ###
        #############

        (topic_namespace, topic_name) = tfs.getTopicName( self.system_root )

        if topic_namespace == None or topic_name == None:
            return (False, "Unable to get topic name")

        self.topic = topic_name

        ##################
        ### QUEUE SIZE ###
        ##################

        queue_size_default_value = 1
        self.queue_size = tfs.getSubscriberQueueSize( self.thread )

        if self.queue_size == None:
            self.queue_size = queue_size_default_value
            log.info("Queue size set to default value: {}".format(self.queue_size))

        ######################
        ### SUBSCRIBER VAR ###
        ######################

        var_subscriber_pub = Variable(self.associated_class)
        var_subscriber_pub.setName( "sub_{}".format(self.name) )
        var_subscriber_pub.setType( ROS_Subscriber( self.associated_class) )
        self.associated_class.addInternalVariable( var_subscriber_pub )

        ###########################
        ### SUBSCRIBER CALLBACK ###
        ###########################

        self.subscriberCallback = Method( self.associated_class )
        self.subscriberCallback.method_name = "{}_callback".format( self.name )
        self.subscriberCallback.return_type = Void( self.associated_class )
        self.subscriberCallback.namespace = self.associated_class.class_name

        input_var = Variable( self.associated_class )
        input_var.setType( self.input_type )
        input_var.setName( "msg" )
        input_var.setIsParameter()

        self.subscriberCallback.addInputParameter( input_var )

        self.subscriberCallback.addMiddleCode("ROS_INFO(\"%s\", {}->data.c_str());".format( input_var.name ))

        comment_source_code = Comment( self.associated_class )
        comment_source_code.setComment( "Source text: {}".format( self.source_text ) )
        self.subscriberCallback.addMiddleCode( comment_source_code )

        self.associated_class.addPrivateMethod( self.subscriberCallback )

        main_thread.prepare.addMiddleCode("{} = handle.subscribe(\"{}\", {}, {}, this);"
                                          .format(var_subscriber_pub.name, self.topic, self.queue_size,
                                                  self.subscriberCallback.getThreadPointer()))

        return (True, "")