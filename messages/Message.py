import os
import systems.SystemsManager as sm
from systems.System import System
import FolderTreeFunctions as folderTree


class Message:
    def __init__(self, namespace, name):
        self.name = name
        self.namespace = namespace

        # Mi prendo il system associato e se questo non esiste ne creo uno nuovo
        self.system = sm.getSystemForNamespace(namespace)

        if self.system is None:
            self.system = System(system_root=None, namespace=namespace)
            sm.addSystem(self.system)

        self.parameters = []
        self.dependencies = []

    def addParameter(self, param):
        self.parameters.append(param)

        if param.type.namespace:
            self.addDependency(param.type.namespace)

    def addDependency(self, dep):
        self.dependencies.append(dep)
        self.dependencies = list(set(self.dependencies))

    ##################
    ### COMPARISON ###
    ##################
    def elementIsInList(self, elem, list):
        for l in list:
            if elem.isEqualTo(l):
                return True
        return False

    def compareList(self, listA, listB):
        return [self.elementIsInList(l, listB) for l in listA].count(False)

    def isEqualTo(self, another_message):
        if self.name != another_message.name:
            return False

        res = self.compareList(self.parameters, another_message.parameters)
        if res != 0:
            return False

        return True

    ##############
    ### SAVING ###
    ##############

    def saveMessageMSG(self):
        system_folder = folderTree.getMessageFolderForSystemFolder(self.system.system_folder)
        output_folder = os.path.join(system_folder, self.getMSGFilename())

        with open(output_folder, 'w+') as file:
            file.write(self.generateCode())

    def getMSGFilename(self):
        return self.name + ".msg"

    def generateCode(self):
        content = ""

        for r in self.parameters:
            content += r.generateCodeForMessageAndService() + "\n"

        # Rimuovo l'ultimo a capo introdotto a fine file
        content = content.rstrip()

        return content
