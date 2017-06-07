import logging
log = logging.getLogger("root")

from threads.AADLThread import AADLThread
from lxml import etree

class Generic(AADLThread):
    def __init__(self, process, thread):
        super().__init__(process, thread)

        log.warning("Thread type NOT recognized: {}".format( self.name ))

    def getDescriptionForComparison(self):
        desc = {}
        desc['type'] = self.type
        return desc

    def generateCode(self):
        return (False, "Tipologia di thread non riconosciuta");