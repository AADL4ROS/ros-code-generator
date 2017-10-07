import XMLTags
import logging
log = logging.getLogger("root")

class LaunchFile():
    def __init__(self, system):
        self.system = system
        self.system_name = self.system.find(XMLTags.tags['TAG_TYPE']).text
        #self.system_name = self.system_name.replace(".", "_")

        self.nodes = []

        self.subsystems = []

        self.system_namespace = None
        try:
            self.system_namespace = self.system.find(XMLTags.tags['TAG_NAMESPACE']).text
        except Exception:
            log.error("Unable to retrieve system namespace")

    def addNode(self, node):
        self.nodes.append(node)

    def hasNode(self, node):
        for n in self.nodes:
            if node.name == n.name:
                return True
        return False

    def retrieveNode(self, node_name):
        for n in self.nodes:
            if node_name == n.name:
                return n
        return None

    def addSubSystem(self, sub_sys):
        self.subsystems.append(sub_sys)

    def generateCode(self):
        text = ""

        text += "<launch>\n"

        for s in self.subsystems:
            text += "\t<include file=\""
            text += "$(find {})/launch/{}.launch\" />\n".format(s.system_namespace, s.system_name)

        for n in self.nodes:
            text += "".join(["\t" + gen_n + "\n" for gen_n in n.generateCode().split("\n")])

        text += "</launch>"

        return text
