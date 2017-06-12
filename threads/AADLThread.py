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

    SUBSCRIBER    = 'subscriber'  # Identifica i thread di tipo publisher

# In questa classe è presente il nome del modulo e della classe che gestisce la creazione di quel
# particolare tipo di thread. Un caso di esempio è il seguente.
# Thread di tipo publisher.
# Il nome del modulo che lo gestisce è threads.Publisher
# Il nome della classe all'interno del modulo è Publisher
class AADLThreadMapping():
    NAME_TO_CLASS = {   AADLThreadType.MAIN_THREAD  : "MainThread",
                        AADLThreadType.PUBLISHER    : "Publisher",
                        AADLThreadType.SUBSCRIBER   : "Subscriber"}

# Classe da cui ereditano tutti i thread
class AADLThread():
    def __init__(self, _process, _thread, _type, _associated_class):
        # AADLProcess a cui un AADLThread fa riferimento
        self.associated_class = _associated_class

        # Processo e thread relativi
        self.process    = _process
        self.thread     = _thread

        # Tipo thread
        self.type       = _type

        # Nome del thread
        self.name       = self.thread.find( XMLTags.tags['TAG_NAME'] ).text

    def populateData(self):
        raise NotImplementedError("populateData deve essere implementata da ogni subclass di Thread")