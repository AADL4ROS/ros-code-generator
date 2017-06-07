import os
import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread
from threads.AADLThread import AADLThreadType
from lxml import etree

import threads.AADLThreadFunctionsSupport as tfs

import datatypes.Datatype as dt

class Publisher(AADLThread):
    def __init__(self, process, thread):
        super().__init__(process, thread)
        self.type = AADLThreadType.PUBLISHER

        # Imposto le path per la lettura del template ed il salvataggio del file
        # finale generato
        self.source_template_path   = os.path.join(self.template_folder, "Publisher.cpp")
        self.source_output_path     = os.path.join(self.output_folder, self.name + ".cpp")

        log.info("Publisher thread {}".format( self.name ) )

    def getDescriptionForComparison(self):
        desc = {}
        desc['type']                            = self.type
        desc['main_thread_prepare']             = self.prepare_source_text
        desc['main_thread_teardown']            = self.teardown_source_text
        desc['main_thread_errorhandler']        = self.errorhandler_source_text
        desc['output_port_datatype']            = self.output_port_datatype
        desc['output_port_datatype_namespace']  = self.output_port_datatype_namespace
        desc['source_text']                     = self.source_text
        desc['frequency']                       = self.frequency_in_hz
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
        aadl_output_port = tfs.getFeatureByName(self.thread, name = "msg")

        if aadl_output_port == None:
            return (False, "Unable to find the default output port named msg")

        (aadl_output_port_datatype_namespace, aadl_output_port_datatype) = tfs.getPortDatatypeByPort( aadl_output_port )

        (self.output_port_datatype_namespace,
         self.output_port_datatype) = dt.getROSDatatypeFromAADLDatatype( (aadl_output_port_datatype_namespace,
                                                                          aadl_output_port_datatype) )

        ###################
        ### Source Text ###
        ###################

        self.source_text = tfs.getSourceText( thread_function )

        if self.source_text == None:
            return (False, "Unable to find property Source_Text")

        ##############
        ### Period ###
        ##############

        (self.period, self.period_unit) = tfs.getPeriod( self.thread )

        if self.period == None or self.period_unit == None:
            return (False, "Unable to find property Period with relative value and unit")

        # Conversione in secondi della frequenza a partire da qualunque unità di misura
        try:
            period_quantity = ureg("{} {}".format(self.period, self.period_unit))
            period_quantity.ito( ureg.second )
            self.frequency_in_hz = 1.0 / period_quantity.magnitude
        except ValueError:
            return (False, "Unable to convert Period in seconds");

        log.info("Period: {} {} -> {} Hz".format( self.period, self.period_unit, self.frequency_in_hz) )
        log.info("Source Text: {}".format(self.source_text) )

        # Leggo il file source template
        with open(self.source_template_path, 'r') as template_file:
            template_source = template_file.read()

        # Sostituisco i placeholder presenti nel file con i valori che ho letto del file AADL
        dict_replacements = {   "{{__DISCLAIMER__}}"    : self.disclaimer,
                                "{{__NODE_NAME__}}"     : self.name,
                                "{{__CLASS_NAME__}}"    : self.name.title(),
                                "{{__FREQUENCY__}}"     : str(int( self.frequency_in_hz )),
                                "{{__DT_NAMESPACE__}}"  : self.output_port_datatype_namespace,
                                "{{__DATATYPE__}}"      : self.output_port_datatype }

        self.output_source = self.replacePlaceholders(template_source, dict_replacements)

        return (True, self.source_output_path)