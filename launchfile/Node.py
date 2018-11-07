import XMLTags
import logging
from launchfile.Remap import Remap
import threads.AADLThreadFunctionsSupport as tfs

log = logging.getLogger("root")


class Node:
    def __init__(self, process):
        self.process = process
        self.name = tfs.getName(process)
        self.type = tfs.getType(process)
        self.namespace = tfs.getNamespace(process)

        self.remap = []
        self.topic_ports = {}

    def addRemap(self, remap):
        self.remap.append(remap)

    def hasPortForTopic(self, port_name):
        for n in self.topic_ports:
            if port_name == n:
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

        (topic_properties_namespace, default_topic_name) = self.getDefaultTopicName(port)
        (topic_properties_namespace, new_topic_name) = self.getTopicName(connection)

        if default_topic_name is None or new_topic_name is None:
            log.warning("No topic remap for port {} of process {}".format(port_name, self.name))
            return True

        # Se ho già controllato quella porta vado oltre
        if self.hasPortForTopic(port_name):
            if self.topic_ports[port_name] == new_topic_name:
                return True
            else:
                log.error("Multiple topics defined for port {} of process {}".format(port_name, self.name))
                return False

        r = Remap(default_topic_name, new_topic_name)

        self.topic_ports[port_name] = new_topic_name

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

        text += "<node name=\"{}\" type=\"{}\" pkg=\"{}\">\n" \
            .format(self.name, self.type, self.namespace)

        for r in self.remap:
            text += "\t{}\n".format(r.generateCode())

        text += "</node>\n"

        return text
