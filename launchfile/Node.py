import XMLTags
import logging
log = logging.getLogger("root")

from launchfile.Remap import Remap

import threads.AADLThreadFunctionsSupport as tfs

class Node():
    def __init__(self, process):
        self.process    = process
        self.name       = process.find(XMLTags.tags['TAG_NAME']).text
        self.type       = process.find(XMLTags.tags['TAG_TYPE']).text
        self.namespace  = process.find(XMLTags.tags['TAG_NAMESPACE']).text

        self.remap = []
        self.topic_ports = []

    def addRemap(self, remap):
        self.remap.append(remap)

    def hasPortForTopic(self, port):
        for n in self.topic_ports:
            if port.find(XMLTags.tags['TAG_NAME']).text == n.find(XMLTags.tags['TAG_NAME']).text:
                return True
        return False

    def addPortForTopicFromPortName(self, port_name, connection):
        try:
            port = self.process.find("./" +
                                     XMLTags.tags['TAG_FEATURES'] + "/" +
                                     XMLTags.tags['TAG_FEATURE'] + "/" +
                                     "[" + XMLTags.tags['TAG_NAME'] + "='" + port_name + "']")
        except Exception:
            return False

        # Se ho già controllato quella porta vado oltre
        if self.hasPortForTopic(port):
            return True

        self.topic_ports.append(port)

        (topic_properties_namespace, default_topic_name) = self.getDefaultTopicName(port)
        (topic_properties_namespace, new_topic_name)     = self.getTopicName(connection)

        if default_topic_name is None or \
            new_topic_name is None:
            return False

        r = Remap(default_topic_name, new_topic_name)
        self.addRemap(r)

        return True

    # Cerco il nome di default del topic associato al node
    def getDefaultTopicName(self, port):
        return tfs.getDefaultTopicName(port)

    # Cerco il nome del topic nuovo associato alla connesione
    def getTopicName(self, connection):
        return tfs.getTopicName(connection)

    def generateCode(self):
        text = ""

        text += "<node name=\"{}\" type=\"{}\" pkg=\"{}\">\n"\
            .format(self.name, self.type, self.namespace)

        for r in self.remap:
            text += "\t{}\n".format( r.generateCode() )

        text += "</node>\n"

        return text

