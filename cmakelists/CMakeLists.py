import os

kCMAKE_LIST_FILENAME = "CMakeLists.txt"

class CMakeLists():
    def __init__(self, system):
        self.system         = system
        self.project_name   = system.namespace

        self.cmake_minimum_req_version = "2.8.3"
        self.use_C11 = True

        self.packages               = []
        self.executables            = []
        self.services               = []
        self.messages               = []
        self.msgs_srvs_dependecies  = []

        # Aggiungo i pacchetti standard
        self.addPackage("roscpp")

    def addPackage(self, lib):
        # Se sto aggiungendo un package tramite una stringa, allora non ho
        # bisogno di chiedere alla libreria il nome del package.
        if isinstance(lib, str):
            pkg = lib
        else:
            pkg = lib.getPackageName()

        # Sto aggiungengo un package che fa riferimento al mio namespace, deriva dalle librerie
        # dei servizi e dei messaggi autogenerate e quindi non lo devo aggiungere al CMakeLists
        if self.project_name == pkg:
            return

        self.packages.append(pkg)

        # Rimuovo eventuali duplicati
        self.packages = list(set( self.packages ))

    def removePackage(self, lib):
        try:
            if isinstance(lib, str):
                pkg = lib
            else:
                pkg = lib.getPackageName()
            self.packages.remove(pkg)
            return True
        except ValueError:
            return False

    # Ogni executable ha la seguente struttura:
    # {
    #   'name' : nome che si vuole dare all'eseguibile
    #   'path' : la path all'eseguibile
    # }
    def addExecutable(self, executable):
        # Controllo nel caso di duplicati
        for e in self.executables:
            if e['name'] == executable['name']:
                return False

        self.executables.append(executable)
        return True

    def addService(self, ser):
        self.services.append(ser.getSRVFilename())

        # Rimuovo eventuali duplicati
        self.services = list(set( self.services ))

        if self.hasMessagesOrServices():
            self.addPackage("message_generation")

        # Aggiungo anche tutte le dipendenze relative al servizio
        for d in ser.dependencies:
            self.addMessageOrServiceDependency(d)

    def addMessage(self, msg):
        self.messages.append(msg.getMSGFilename())

        # Rimuovo eventuali duplicati
        self.messages = list(set( self.messages ))

        if self.hasMessagesOrServices():
            self.addPackage("message_generation")

        # Aggiungo anche tutte le dipendenze relative al messaggio
        for d in msg.dependencies:
            self.addMessageOrServiceDependency(d)

    def addMessageOrServiceDependency(self, dep):
        self.msgs_srvs_dependecies.append(dep)

        # Rimuovo eventuali duplicati
        self.msgs_srvs_dependecies = list(set(self.msgs_srvs_dependecies))

    def hasMessagesOrServices(self):
        return (len(self.messages) > 0 or len(self.services) > 0)

    def generateHeaderCommentWithText(self, text):
        number_of_hashtag_before_text = 3

        # La formula vuol dire questo:
        #   devi mettere in una riga un numero di # (hashtag) pari alla lunghezza
        #   del testo del commento più il numero di hastag prima e dopo il testo
        #   (ecco spiegato il *2) più 2 cancelletti che sono gli spazi prima e dopo
        #   il testo. Tanta complicazione per nulla, lo so
        total_hashtag_length = (len(text) + number_of_hashtag_before_text * 2 + 2 )

        comment = "\n"
        comment += "#" * total_hashtag_length
        comment += "\n"
        comment += "#" * number_of_hashtag_before_text
        comment += " {} ".format(text)
        comment += "#" * number_of_hashtag_before_text
        comment += "\n"
        comment += "#" * total_hashtag_length
        comment += "\n"
        return comment

    def saveCMakeList(self):
        output_folder = os.path.join(self.system.system_folder, kCMAKE_LIST_FILENAME)
        with open(output_folder, 'w+') as file:
            file.write(self.generateCode())

    def generateCode(self):
        text = ""
        text += self.generateHeaderCommentWithText("Auto-generated CMakeList")

        # Inizio
        text += "cmake_minimum_required(VERSION {})\n".format(self.cmake_minimum_req_version)
        text += "project({})\n".format(self.project_name)

        if self.use_C11:
            text += "set( CMAKE_CXX_STANDARD 11 )\n"

        # Packages
        text += self.generateHeaderCommentWithText("Packages")
        text += "find_package(catkin REQUIRED COMPONENTS\n"

        # Aggiungo anche i package relativi alle dipendenze dei messaggi e dei servizi
        for p in set(self.packages + self.msgs_srvs_dependecies):
            text += "\t{}\n".format(p)

        text += ")\n"

        # Services
        if len(self.services) > 0:
            text += self.generateHeaderCommentWithText("Services")
            text += "add_service_files(\n\tFILES\n"
            for s in self.services:
                text += "\t{}\n".format(s)
            text += ")\n"

        # Messages
        if len(self.messages) > 0:
            text += self.generateHeaderCommentWithText("Messages")
            text += "add_message_files(\n\tFILES\n"
            for m in self.messages:
                text += "\t{}\n".format(m)
            text += ")\n"

        # Generate Message
        if len(self.msgs_srvs_dependecies) > 0:
            text += self.generateHeaderCommentWithText("MSGS and SRVS Dependencies")
            text += "generate_messages(\n\tDEPENDENCIES\n"
            for d in self.msgs_srvs_dependecies:
                text += "\t{}\n".format(d)
            text += ")\n"

        # Build HARDCODED
        text += self.generateHeaderCommentWithText("Build")

        # Aggiungo la chiamata a catkin_package
        text += "catkin_package()"

        text += "include_directories(\n"
        text += "\t${catkin_INCLUDE_DIRS}\n"
        text += "\t${ros_base_INCLUDE_DIRS}\n"
        text += "\tinclude\n"
        text += ")\n"

        for e in self.executables:
            text += "add_executable({} {})\n".format(e['name'], e['path'])

        for e in self.executables:
            text += "target_link_libraries({}\n".format(e['name'])
            text += "\t${catkin_LIBRARIES}\n"
            text += "\t${ros_base_LIBRARIES}\n"
            text += ")\n"

        return text