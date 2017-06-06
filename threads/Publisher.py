import os
import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread
from lxml import etree

import threads.AADLThreadFunctionsSupport as tfs

class Publisher(AADLThread):
    def __init__(self, process, thread):
        super().__init__(process, thread)

        # Imposto le path per la lettura del template ed il salvataggio del file
        # finale generato
        self.source_template_path   = os.path.join(self.template_folder, "Publisher.cpp")
        self.source_output_path     = os.path.join(self.output_folder, self.name + ".cpp")

        log.info("Publisher thread {}".format( self.name ) )

    def generate_code(self):
        ###################
        ### MAIN THREAD ###
        ###################

        # Ottento tutti i dati relativi al main thread
        self.populateMainThreadData()

        if self.main_thread == None:
            return (False, "Unable to find the right Main Thread")

        self.prepare_source_text = tfs.getSourceText( self.prepare )

        if self.prepare_source_text != None:
            print("Aggiungerò il contenuto di questo file nel mio thread")

        # Ottengo le informazioni necessarie per i thread di tipo Publisher:
        # - Source Text
        # - Period

        thread_function = tfs.getSubprogram( self.thread )
        if thread_function == None:
            return (False, "Unable to find the right Subprogram")

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
                                "{{__FREQUENCY__}}"     : str(int( self.frequency_in_hz )) }

        output_source = self.replace_placeholders(template_source, dict_replacements)

        # Salvo il file finale
        with open(self.source_output_path, 'w+') as file:
            file.write( output_source )

        return (True, self.source_output_path)