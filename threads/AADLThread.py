# Tipologie di thread: definisce una corrispondenza fra il nome nel package
# AADL e la tipologia di thread gestita dal code generator
class AADLThreadType():
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
    def __init__(self, process, thread, tags):
        self.process    = process
        self.thread     = thread
        self.tags       = tags

        self.name = process.find(self.tags['TAG_NAME']).text

    def generate_code(self):
        raise NotImplementedError("generate_code deve essere implementata da ogni subclass di Thread");