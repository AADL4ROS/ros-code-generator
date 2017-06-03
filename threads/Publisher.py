import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread
from lxml import etree

class Publisher(AADLThread):
    def __init__(self, process, thread, tags):
        super().__init__(process, thread, tags)

        log.info("Publisher thread {}".format( self.name ) )

    def generate_code(self):
        # Ottengo le informazioni necessarie per i thread di tipo Publisher

        thread_function = self.thread.find("./" +
                                            self.tags['TAG_SUBCOMPONENTS'] + "/" +
                                                self.tags['TAG_SUBCOMPONENT'] + "/" +
                                                    "[" + self.tags['TAG_CATEGORY'] + "='subprogram']");
        # Source Text
        try:
            source_text_property = thread_function.find("./" +
                                                    self.tags['TAG_PROPERTIES'] + "/" +
                                                        self.tags['TAG_PROPERTY'] + "/" +
                                                            "[" + self.tags['TAG_PROPERTY_NAME'] + "='Source_Text']")

            self.source_text = source_text_property.find(self.tags['TAG_PROPERTY_VALUE']).text
        except AttributeError:
            return (False, "Unable to find property Source_Text");

        # Period
        try:
            period_property = self.thread.find("./" +
                                                self.tags['TAG_PROPERTIES'] + "/" +
                                                    self.tags['TAG_PROPERTY'] + "/" +
                                                        "[" + self.tags['TAG_PROPERTY_NAME'] + "='Period']")
            self.period      = period_property.find(self.tags['TAG_PROPERTY_VALUE']).text
            self.period_unit = period_property.find(self.tags['TAG_PROPERTY_UNIT']).text
        except AttributeError:
            return (False, "Unable to find property Period with relative value and unit");

        log.info("Period: {} {}".format( self.period, self.period_unit) )
        log.info("Source Text: {}".format(self.source_text) )


        

        return (True, "");