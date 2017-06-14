import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread
from threads.AADLThread import AADLThreadType
from lxml import etree

import threads.AADLThreadFunctionsSupport as tfs

import datatypes.DatatypeFromPort as dt

from datatypes.Type import Int, Double, Void, ROS_TimerEvent, ROS_Timer, ROS_Publisher

from variables.Variable import Variable
from methods.Method import Method
from comments.Comment import Comment

class Publisher(AADLThread):
    def __init__(self, _process, _thread, _associated_class):
        super().__init__(_process, _thread, AADLThreadType.PUBLISHER, _associated_class)
        log.info("Publisher thread {}".format(self.name))

        # Parametri del Publisher
        self.source_text        = None
        self.frequency_in_hz    = None
        self.topic              = None
        self.publisherCallback  = None
        self.output_type        = None

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

        ###################
        ### Output Port ###
        ###################

        # Ottengo la porta in output per i thread di tipo Publisher
        aadl_output_port = tfs.getFeatureByName(self.thread, name = "msg")

        if aadl_output_port == None:
            return (False, "Unable to find the default output port named msg")

        (aadl_output_port_datatype_namespace, aadl_output_port_datatype) = tfs.getPortDatatypeByPort( aadl_output_port )

        self.output_type = dt.getROSDatatypeFromAADLDatatype( (aadl_output_port_datatype_namespace, aadl_output_port_datatype),
                                                                self.associated_class )
        ###################
        ### Source Text ###
        ###################

        self.source_text = tfs.getSourceText( thread_function )

        if self.source_text == None:
            return (False, "Unable to find property Source_Text")

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
        except ValueError:
            return (False, "Unable to convert Period in seconds")

        param_freq = Variable( self.associated_class )
        param_freq.setName( "frequency_{}".format(self.name) )
        param_freq.setType( Int( self.associated_class ))
        self.associated_class.addParameter( param_freq )

        #############
        ### TOPIC ###
        #############

        #@TODO Aggiungere supporto al topic
        self.topic = "/chatter"

        #####################
        ### STARTING TIME ###
        #####################

        var_starting_time = Variable(self.associated_class)
        var_starting_time.setName( "starting_time_{}".format(self.name) )
        var_starting_time.setType( Double(self.associated_class) )
        self.associated_class.addVariable( var_starting_time )

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

        self.publisherCallback.addMiddleCode("std_msgs::String msg;\n"
                                             "std::stringstream ss;\n"
                                             "ss << \"current time: \" << (ros::Time::now().toSec() - vars.{});\n"
                                             "msg.data = ss.str().c_str();\n"
                                             "{}.publish(msg);".format(var_starting_time.name, var_publisher_pub.name ))


        comment_source_code = Comment( self.associated_class )
        comment_source_code.setComment( "Source text: {}".format( self.source_text ) )
        self.publisherCallback.addMiddleCode( comment_source_code )

        self.associated_class.addPrivateMethod( self.publisherCallback )

        main_thread.prepare.addTopCode( "params.{} = {};".format(param_freq, self.frequency_in_hz) )
        main_thread.prepare.addMiddleCode("handle.getParam(\"{}\", params.{});".format(param_freq.name, param_freq.name))

        main_thread.prepare.addMiddleCode("{} = handle.advertise < {} > (\"{}\", 10);"
                                          .format(var_publisher_pub.name, self.output_type.generateCode(), self.topic))

        main_thread.prepare.addMiddleCode("{} = handle.createTimer(ros::Duration(1/params.{}), {}, this);"
                                            .format(var_timer_pub.name, param_freq.name, self.publisherCallback.getThreadPointer() ))

        main_thread.prepare.addMiddleCode("vars.{} = ros::Time::now().toSec();".format(var_starting_time.name))

        return (True, "")