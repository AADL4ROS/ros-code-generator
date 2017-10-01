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

    PUBLISHER   = 'publisher.impl'   # Identifica i thread di tipo publisher

    SUBSCRIBER    = 'callback.impl'  # Identifica i thread di tipo publisher

    SUBSCRIBER_PUBLISHER = 'call_pub.impl'  # Identifica i thread di tipo subscriber-publisher, thread che ripetono un
                                            # messaggio ricevuto in input su un topic su un topic di output dopo averlo
                                            # eventualmente manipolato

# In questa classe è presente il nome del modulo e della classe che gestisce la creazione di quel
# particolare tipo di thread. Un caso di esempio è il seguente.
# Thread di tipo publisher.
# Il nome del modulo che lo gestisce è threads.Publisher
# Il nome della classe all'interno del modulo è Publisher
class AADLThreadMapping():
    NAME_TO_CLASS = {   AADLThreadType.MAIN_THREAD          : "MainThread",
                        AADLThreadType.PUBLISHER            : "Publisher",
                        AADLThreadType.SUBSCRIBER           : "Subscriber",
                        AADLThreadType.SUBSCRIBER_PUBLISHER : "SubscriberPublisher"}

# Classe da cui ereditano tutti i thread
class AADLThread():
    def __init__(self, _system_root, _process, _thread, _type, _associated_class):
        # AADLProcess a cui un AADLThread fa riferimento
        self.associated_class = _associated_class

        # Processo e thread relativi
        self.system_root    = _system_root
        self.process        = _process
        self.thread         = _thread

        # Tipo thread
        self.type       = _type

        # Nome del thread
        self.name       = tfs.getName( self.thread )

    # getTopicName
    # A partire dal processo attuale, cerca il nome del topic associato alla connessione
    # Ha bisogno del nome della porta specificato fra le features del processo ed il nome
    # del processo. I parametri input, output definiscono se la porta da cercare è una porta
    # di input (nel caso di un Subscriber), oppure di output (nel caso di un Publisher)
    def getTopicName(self, process_port_name, input=False, output=False):
        process_name = tfs.getName(self.process)

        connections = tfs.getAllConnectionsPerPort(self.system_root, process_name, process_port_name,
                                                    input=input,
                                                    output=output)

        names = []
        for c in connections:
            (topic_namespace, topic_name) = tfs.getTopicName(c)

            if topic_namespace == None or topic_name == None:
                return (False, "Unable to get topic name")

            names.append(topic_name)

        names = list(set(names)) # Rimuovo i duplicati

        if len(names) < 1:
            return (False, "No topic name defined")

        if len(names) > 1:
            return (False, "Multiple topic names defined")

        self.topic = names[0]

        return (True, "")

    def populateData(self):
        raise NotImplementedError("populateData deve essere implementata da ogni subclass di Thread")