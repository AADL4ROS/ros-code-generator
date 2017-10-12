import threads.AADLThreadFunctionsSupport as tfs
import os

kCMAKE_LIST_FILENAME = "CMakeLists.txt"

class CMakeLists():
    def __init__(self, system_root):
        self.project_name = tfs.getNamespace(system_root)

        self.cmake_minimum_req_version = "2.8.3"

        self.packages       = []
        self.executables    = []

        # Aggiungo i pacchetti standard
        self.addPackage("roscpp")

    def addPackage(self, lib):
        # Se sto aggiungendo un package tramite una stringa, allora non ho
        # bisogno di chiedere alla libreria il nome del package.
        if isinstance(lib, str):
            pkg = lib
        else:
            pkg = lib.getPackageName()

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

    def saveCMakeListInFolder(self, system_folder):
        output_folder = os.path.join(system_folder, kCMAKE_LIST_FILENAME)
        with open(output_folder, 'w+') as file:
            file.write(self.generateCode())

    def generateCode(self):
        text = ""
        text += self.generateHeaderCommentWithText("Auto-generated CMakeList")

        # Inizio
        text += "cmake_minimum_required(VERSION {})\n".format(self.cmake_minimum_req_version)
        text += "project({})\n".format(self.project_name)

        # Packages
        text += self.generateHeaderCommentWithText("Packages")
        text += "find_package(catkin REQUIRED COMPONENTS\n"

        for p in self.packages:
            text += "\t{}\n".format(p)

        text += ")\n"

        # Build HARDCODED
        text += self.generateHeaderCommentWithText("Build")
        text += "include_directories(\n"
        text += "\t${catkin_INCLUDE_DIRS}\n"
        text += "\t${ros_base_INCLUDE_DIRS}\n"
        text += ")\n"

        for e in self.executables:
            text += "add_executable({} {})\n".format(e['name'], e['path'])

        for e in self.executables:
            text += "target_link_libraries({}\n".format(e['name'])
            text += "\t${catkin_LIBRARIES}\n"
            text += "\t${ros_base_LIBRARIES}\n"
            text += ")\n"

        return text