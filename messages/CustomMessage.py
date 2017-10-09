from CObject import CObject
from asn1tools.parser import parse_file
from messages.Message import Message

class CustomMessage(CObject):
    def __init__(self, _associated_class = None, asn1_source = None):
        super().__init__(_associated_class)
        self.asn1_source = asn1_source
        self.custom_msgs = []

    def parseASN1Source(self):
        if self.asn1_source == None:
            return False

        # @TODO: il percorso del file AADL è lo stesso del file XML
        ocarina_ros_path = "../ocarina-ros/"
        parsed_asn = parse_file(os.path.join(ocarina_ros_path, self.asn1_source))

        message = parsed_asn['Message']

        # In ogni tipo nella struttura messaggio è definito un messaggio.
        # Uno potrebbe definire più tipi di messaggio nello stesso file,
        # noi li generiamo tutti (anche se sarebbe meglio definire un file ASN.1 per ogni
        # tipo di messaggio)
        for t in message['types']:
            tmp_msg = Message(t, t['members'], self.associated_class)
            pass

    def generateCode(self):
        pass