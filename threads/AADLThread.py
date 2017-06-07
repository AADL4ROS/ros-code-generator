import os
import datetime
import XMLTags
import threads.AADLThreadFunctionsSupport as tfs

# Tipologie di thread: definisce una corrispondenza fra il nome nel package
# AADL e la tipologia di thread gestita dal code generator
class AADLThreadType():
    MAIN_THREAD = 'main_thread' # Identifica il thread base da cui sono composti tutti gli elementi, questo thread è
                                # passato al costruttore delle altre tipologie di thread in quanto contiene riferimenti
                                # alle funzioni base come prepare, tearDown e errorHandling

    PUBLISHER   = 'publisher'   # Identifica i thread di tipo publisher

# In questa classe è presente il nome del modulo e della classe che gestisce la creazione di quel
# particolare tipo di thread. Un caso di esempio è il seguente.
# Thread di tipo publisher.
# Il nome del modulo che lo gestisce è threads.Publisher
# Il nome della classe all'interno del modulo è Publisher
class AADLThreadMapping():
    NAME_TO_CLASS = { AADLThreadType.PUBLISHER : "Publisher" }

# Classe da cui ereditano tutti i thread
class AADLThread():
    def __init__(self, process, thread):
        dir = os.path.dirname(__file__)
        self.template_folder = os.path.join(dir, "..", "template", "threads")
        self.output_folder   = os.path.join(dir, "..", "src")
        self.output_source   = None

        try:
            os.mkdir( self.output_folder )
        except FileExistsError:
            pass

        # Processo e thread relaviti
        self.process        = process
        self.thread         = thread

        # Main Thread
        self.main_thread    = None
        self.prepare        = None
        self.tearDown       = None
        self.errorHandler   = None

        # Proprietà thread
        self.type       = None
        self.name       = process.find(XMLTags.tags['TAG_NAME']).text
        self.disclaimer = self.generateDisclaimer()

    def populateMainThreadData(self):
        # Ottengo il main thread
        self.main_thread = tfs.getMainThread(self.process)

        if self.main_thread == None:
            return

        (self.prepare, self.tearDown, self.errorHandler) = tfs.getMainThreadFunctions(self.main_thread)

    def generateCode(self):
        raise NotImplementedError("generateCode deve essere implementata da ogni subclass di Thread")

    def getDescriptionForComparison(self):
        raise NotImplementedError("getDescriptionForComparison deve essere implementata da ogni subclass di Thread")

    def generateDisclaimer(self):
        today = datetime.datetime.now()
        generated_on = today.strftime("%d/%m/%Y %H:%M:%S")

        disclaimer  = "/**\n"
        disclaimer += " * Node {}\n".format(self.name)
        disclaimer += " * File auto-generated on {}\n".format(generated_on)
        disclaimer += " */\n"

        return disclaimer

    def replacePlaceholders(self, text, dict_replacements):
        text_output = text
        for key, value in dict_replacements.items():
            text_output = text_output.replace(key, value)
        return text_output

    def saveOutputSource(self):
        # Salvo il file finale
        with open(self.source_output_path, 'w+') as file:
            file.write(self.output_source)