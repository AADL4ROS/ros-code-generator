import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs
import messages.MessageFunctionSupport as mfs

import datatypes.DatatypeConversion as dt

from datatypes.Type import Void, ROS_Subscriber

from variables.Variable import Variable
from methods.Method import Method
from comments.Comment import Comment
from datatypes.Type import Type
from libraries.Library import Library

class Subscriber(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
        log.info("Subscriber thread {}".format(self.name))

        self.input_port_name = "msg"

        # Parametri del Subscriber
        self.process_port       = None
        self.source_text_file   = None
        self.topic              = None
        self.subscriberCallback = None
        self.input_type         = None
        self.asn1_source_file   = None
        self.queue_size         = None

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

        ############################
        ### TRANSFORMATION FRAME ###
        ############################

        # Controllo l'uso del Transformation Frame
        self.thread_uses_tf = self.setUsesTransformationFrame()

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

        self.process_port = tfs.getFeatureByName(self.process, name=source_name)

        if self.process_port == None:
            return (False, "Unable to find the process input port name feature")

        (aadl_namespace, aadl_type) = tfs.getPortDatatypeByPort(self.process_port)
        if aadl_namespace == None or aadl_type == None:
            return (False, "Unable to identify process port type")

        # Controllo se c'è un file ASN.1 associato alla porta. Se c'è allora il tipo di messaggio
        # è custom e lo dovrò generare, mentre se non c'è allora è un messaggio standard ROS
        port_data_info = tfs.getPortDataInfo(self.process_port)
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

        ###################
        ### Source Text ###
        ###################

        self.source_text_file = self.createSourceTextFileFromSourceText(tfs.getSourceText(thread_function),
                                                                        tfs.getSourceName(thread_function))

        if self.source_text_file == None:
            return (False, "Unable to find property Source_Text or Source_Name")

        self.source_text_file.uses_tf = self.thread_uses_tf

        #############
        ### TOPIC ###
        #############

        (status, desc) = self.getDefaultTopicName(self.input_port_name, input=True)
        if status == False:
            return (status, desc)

        ##################
        ### QUEUE SIZE ###
        ##################

        queue_size_default_value = 1
        self.queue_size = tfs.getSubscriberQueueSize( self.thread, port_name=self.input_port_name )

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
        self.source_text_file.addFunctionParameter( input_var )

        # Aggiungo la chiamata alla funzione custome
        if self.source_text_file != None:
            code = "{};".format(self.source_text_file.generateInlineCode())
            self.subscriberCallback.addMiddleCode(code)

        self.associated_class.addPrivateMethod( self.subscriberCallback )

        main_thread.prepare.addMiddleCode("{} = handle.subscribe(\"{}\", {}, {}, this);"
                                          .format(var_subscriber_pub.name, self.topic, self.queue_size,
                                                  self.subscriberCallback.getThreadPointer()))

        return (True, "")