import os
import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread
from threads.AADLThread import AADLThreadType
from lxml import etree

import threads.AADLThreadFunctionsSupport as tfs

import datatypes.DatatypeFromPort as dt

class Subscriber(AADLThread):
    def __init__(self, process, thread):
        super().__init__(process, thread)
        self.type = AADLThreadType.SUBSCRIBER

        # Imposto le path per la lettura del template ed il salvataggio del file
        # finale generato
        self.source_template_path   = os.path.join(self.template_folder, "Subscriber.cpp")
        self.source_output_path     = os.path.join(self.output_folder, self.name + ".cpp")

        log.info("Publisher thread {}".format( self.name ) )

    def getDescriptionForComparison(self):
        desc = {}
        desc['type']                            = self.type
        desc['main_thread_prepare']             = self.prepare_source_text
        desc['main_thread_teardown']            = self.teardown_source_text
        desc['main_thread_errorhandler']        = self.errorhandler_source_text
        desc['input_port_datatype']             = self.input_port_datatype
        desc['input_port_datatype_namespace']   = self.input_port_datatype_namespace
        desc['source_text']                     = self.source_text
        return desc

    def generateCode(self):
        ###################
        ### MAIN THREAD ###
        ###################

        # Ottento tutti i dati relativi al main thread
        self.populateMainThreadData()

        if self.main_thread == None:
            return (False, "Unable to find the right Main Thread")

        self.prepare_source_text        = tfs.getSourceText( self.prepare )
        self.teardown_source_text       = tfs.getSourceText( self.tearDown )
        self.errorhandler_source_text   = tfs.getSourceText( self.errorHandler )

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
        aadl_input_port = tfs.getFeatureByName(self.thread, name = "msg")

        if aadl_input_port == None:
            return (False, "Unable to find the default input port named msg")

        (aadl_input_port_datatype_namespace, aadl_input_port_datatype) = tfs.getPortDatatypeByPort( aadl_input_port )

        (self.input_port_datatype_namespace,
         self.input_port_datatype,
         input_port_datatype_include) = dt.getROSDatatypeFromAADLDatatype( (aadl_input_port_datatype_namespace,
                                                                            aadl_input_port_datatype) )

        # Il tipo del messaggio inviato va anche importato
        commento_import_datatype = self.generateCommentFromString("Automatically imported from message datatype")
        datatype_include = self.generateInclude( input_port_datatype_include )

        ###################
        ### Source Text ###
        ###################

        self.source_text = tfs.getSourceText( thread_function )

        if self.source_text == None:
            return (False, "Unable to find property Source_Text")

        log.info("Source Text: {}".format(self.source_text) )

        # Leggo il file source template
        with open(self.source_template_path, 'r') as template_file:
            template_source = template_file.read()

        # Sostituisco i placeholder presenti nel file con i valori che ho letto del file AADL
        dict_replacements = {   "{{__DISCLAIMER__}}"        : self.disclaimer,
                                "{{__NODE_NAME__}}"         : self.name,
                                "{{__CLASS_NAME__}}"        : self.name.title(),
                                "{{__DT_NAMESPACE__}}"      : self.input_port_datatype_namespace,
                                "{{__DATATYPE__}}"          : self.input_port_datatype,
                                "{{__DATATYPE_INCLUDES__}}" : commento_import_datatype + datatype_include}

        self.output_source = self.replacePlaceholders(template_source, dict_replacements)

        return (True, self.source_output_path)