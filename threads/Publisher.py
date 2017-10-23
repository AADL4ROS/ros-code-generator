import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread

import threads.AADLThreadFunctionsSupport as tfs
import messages.MessageFunctionSupport as mfs

import datatypes.DatatypeConversion as dt

from datatypes.Type import ROSBase_PointerToTransformationFrames, Void, ROS_TimerEvent, ROS_Timer, ROS_Publisher

from variables.Variable import Variable
from methods.Method import Method
from comments.Comment import Comment
from datatypes.Type import Type
from libraries.Library import Library, ROSBase_TF_Interface

class Publisher(AADLThread):
    def __init__(self, _system_root, _process, _thread, _associated_class):
        super().__init__(_system_root, _process, _thread, _associated_class)
        log.info("Publisher thread {}".format(self.name))

        self.output_port_name = "msg"

        # Parametri del Publisher
        self.process_port       = None
        self.source_text_file   = None
        self.frequency_in_hz    = None
        self.period_in_seconds  = None
        self.topic              = None
        self.publisherCallback  = None
        self.output_type        = None
        self.asn1_source_file   = None
        self.custom_message     = None

    def populateData(self):
        main_thread = self.associated_class.getMainThread()

        if main_thread == None:
            return (False, "Unable to get the Main Thread")

        # Ottengo le informazioni necessarie per i thread di tipo Publisher:
        # - Source Text
        # - Period

        thread_function = tfs.getSubprogram( self.thread )
        if thread_function == None:
            return (False, "Unable to find the right Subprogram")

        ############################
        ### TRANSFORMATION FRAME ###
        ############################

        # Controllo l'uso del Transformation Frame
        self.thread_uses_tf = self.setUsesTransformationFrame()

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

        self.process_port = tfs.getFeatureByName(self.process, name=dest_name)
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


        ###################
        ### Source Text ###
        ###################

        self.source_text_file = self.createSourceTextFileFromSourceText(tfs.getSourceText( thread_function ),
                                                                        tfs.getSourceName( thread_function ))

        if self.source_text_file == None:
            return (False, "Unable to find property Source_Text or Source_Name")

        self.source_text_file.uses_tf = self.thread_uses_tf
        self.source_text_file.setFunctionType(self.output_type)

        #################
        ### FREQUENCY ###
        #################

        (period, period_unit) = tfs.getPeriod( self.thread )

        if period == None or period_unit == None:
            return (False, "Unable to find property Period with relative value and unit")

        # Conversione in secondi della frequenza a partire da qualunque unità di misura
        try:
            period_quantity = ureg("{} {}".format(period, period_unit))
            period_quantity.ito( ureg.second )
            self.frequency_in_hz = 1.0 / period_quantity.magnitude
            self.period_in_seconds = period_quantity.magnitude
        except ValueError:
            return (False, "Unable to convert Period in seconds")

        # param_freq = Variable( self.associated_class )
        # param_freq.setName( "frequency_{}".format(self.name) )
        # param_freq.setType( Int( self.associated_class ))
        # self.associated_class.addParameter( param_freq )

        #############
        ### TOPIC ###
        #############

        (status, desc) = self.getDefaultTopicName(self.output_port_name, output=True)
        if status == False:
            return (status, desc)

        #####################
        ### PUBLISHER VAR ###
        #####################

        var_publisher_pub = Variable(self.associated_class)
        var_publisher_pub.setName( "pub_{}".format(self.name) )
        var_publisher_pub.setType( ROS_Publisher( self.associated_class) )
        self.associated_class.addInternalVariable( var_publisher_pub )

        #######################
        ### PUBLISHER TIMER ###
        #######################

        var_timer_pub = Variable(self.associated_class)
        var_timer_pub.setName("timer_{}".format(self.name))
        var_timer_pub.setType(ROS_Timer(self.associated_class))
        self.associated_class.addInternalVariable(var_timer_pub)

        ##########################
        ### PUBLISHER CALLBACK ###
        ##########################

        self.publisherCallback = Method( self.associated_class )
        self.publisherCallback.method_name = "{}_callback".format( self.name )
        self.publisherCallback.return_type = Void( self.associated_class )
        self.publisherCallback.namespace = self.associated_class.class_name

        input_par = Variable( self.associated_class )
        input_par.setIsParameter()
        input_par.setType( ROS_TimerEvent( self.associated_class ) )
        input_par.setName("")
        self.publisherCallback.addInputParameter( input_par )

        # Aggiungo la chiamata alla funzione custome
        if self.source_text_file != None:
            code = "{}.publish({});".format(var_publisher_pub.name,
                                           self.source_text_file.generateInlineCode())
            self.publisherCallback.addMiddleCode(code)

        self.associated_class.addPrivateMethod( self.publisherCallback )

        main_thread.prepare.addMiddleCode("{} = handle.advertise < {} > (\"{}\", 10);"
                                          .format(var_publisher_pub.name, self.output_type.generateCode(), self.topic))

        main_thread.prepare.addMiddleCode("{} = handle.createTimer(ros::Duration({}), {}, this);"
                                            .format(var_timer_pub.name, self.period_in_seconds,
                                                    self.publisherCallback.getThreadPointer() ))

        return (True, "")