import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread
from lxml import etree

class Generic(AADLThread):
    def __init__(self, process, thread, tags):
        super().__init__(process, thread, tags)
        log.warning("Thread type NOT recognized")
        log.info("Generic thread {}".format( self.name ) )

    def generate_code(self):
        return (False, "Tipologia di thread non riconosciuta");