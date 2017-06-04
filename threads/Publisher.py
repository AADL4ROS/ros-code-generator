import os
import logging
log = logging.getLogger("root")

from pint import UnitRegistry
ureg = UnitRegistry()

from threads.AADLThread import AADLThread
from lxml import etree

class Publisher(AADLThread):
    def __init__(self, process, thread, tags):
        super().__init__(process, thread, tags)

        # Imposto le path per la lettura del template ed il salvataggio del file
        # finale generato
        self.source_template_path   = os.path.join(self.template_folder, "Publisher.cpp")
        self.source_output_path     = os.path.join(self.output_folder, self.name + ".cpp")

        log.info("Publisher thread {}".format( self.name ) )

    def generate_code(self):
        # Ottengo le informazioni necessarie per i thread di tipo Publisher:
        # - Source Text
        # - Period

        thread_function = self.thread.find("./" +
                                            self.tags['TAG_SUBCOMPONENTS'] + "/" +
                                                self.tags['TAG_SUBCOMPONENT'] + "/" +
                                                    "[" + self.tags['TAG_CATEGORY'] + "='subprogram']");
        ###################
        ### Source Text ###
        ###################
        try:
            source_text_property = thread_function.find("./" +
                                                        self.tags['TAG_PROPERTIES'] + "/" +
                                                            self.tags['TAG_PROPERTY'] + "/" +
                                                                "[" + self.tags['TAG_PROPERTY_NAME'] + "='Source_Text']")

            self.source_text = source_text_property.find(self.tags['TAG_PROPERTY_VALUE']).text
        except AttributeError:
            return (False, "Unable to find property Source_Text");

        ##############
        ### Period ###
        ##############
        try:
            period_property = self.thread.find("./" +
                                                self.tags['TAG_PROPERTIES'] + "/" +
                                                    self.tags['TAG_PROPERTY'] + "/" +
                                                        "[" + self.tags['TAG_PROPERTY_NAME'] + "='Period']")
            self.period      = period_property.find(self.tags['TAG_PROPERTY_VALUE']).text
            self.period_unit = period_property.find(self.tags['TAG_PROPERTY_UNIT']).text
        except AttributeError:
            return (False, "Unable to find property Period with relative value and unit");

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

        return (True, self.source_output_path);