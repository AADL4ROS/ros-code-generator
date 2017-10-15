import os

class Service():
    def __init__(self, name):
        self.name = name
        self.requests = []
        self.responses = []
        self.dependencies = []

    def addRequest(self, req):
        self.requests.append(req)

        if req.type.namespace != None:
            self.addDependency(req.type.namespace)

    def addResponse(self, res):
        self.responses.append(res)

        if res.type.namespace != None:
            self.addDependency(res.type.namespace)

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

    def isEqualTo(self, another_service):
        if self.name != another_service.name:
            return False

        res = self.compareList(self.requests, another_service.requests)
        if res != 0:
            return False

        res = self.compareList(self.responses, another_service.responses)
        if res != 0:
            return False

        return True

    def saveServiceSRVInFolder(self, system_srv_folder):
        output_folder = os.path.join(system_srv_folder, self.getSRVFilename())
        with open(output_folder, 'w+') as file:
            file.write(self.generateCode())

    def getSRVFilename(self):
        return self.name + ".srv"

    def generateCode(self):
        content = ""

        for r in self.requests:
            content += r.generateCodeForMessageAndService() + "\n"

        # Separazione REQUEST and RESPONSE
        content += "---\n"

        for r in self.responses:
            content += r.generateCodeForMessageAndService() + "\n"

        # Rimuovo l'ultimo a capo introdotto a fine file
        content = content.rstrip()

        return content