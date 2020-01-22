import XMLTags
import logging
import folder_tree_functions as folderTree
import os

log = logging.getLogger("root")


class LaunchFile:
    def __init__(self, system):
        self.system = system

        self.system_root = self.system.system_root
        self.system_name = self.system_root.find(XMLTags.tags['TAG_TYPE']).text

        self.nodes = []

        self.subsystems = []

        self.system_namespace = None
        try:
            self.system_namespace = self.system.namespace
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

    def saveLaunchFile(self):
        output_folder = folderTree.getLaunchFolderForSystemFolder(self.system.system_folder)

        filename = "{}.launch".format(self.system_name)
        source_output_path = os.path.join(output_folder, filename)

        with open(source_output_path, 'w+') as file:
            file.write(self.generateCode())

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
