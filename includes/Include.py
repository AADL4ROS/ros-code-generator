from CObject import CObject
import FolderTreeFunctions as folderTree
import os

class Include(CObject):
    def __init__(self, _associated_class, file_name):
        super().__init__( _associated_class )

        (name, ext) = os.path.splitext(file_name)
        self.file_name = name

        # @TODO: Gestione dei file già compilati
        #if ext == "so":
        #    pass

        self.libraries = []
        self.uses_tf = False

    def addLibrary(self, lib):
        for l in self.libraries:
            if l.isEqualTo(lib):
                return

        # Piccolo hack per non aggiungere mai la std/String.h
        if lib.path.lower() == "std/String.h".lower():
            return

        self.libraries.append(lib)

    def getSourceLibraryPath(self):
        return "{}/{}.h".format(self.associated_class.system.namespace, self.file_name)

    def saveFile(self):
        include_folder = folderTree.getIncludeFolderForSystemFolder( self.associated_class.system.system_folder )

        file_name_with_extension = "{}.h".format(self.file_name)

        output_folder = os.path.join(include_folder, self.associated_class.system.namespace, file_name_with_extension)

        with open(output_folder, 'w+') as file:
            file.write(self.generateCode())