import XMLTags
import logging
log = logging.getLogger("root")

class Remap():
    def __init__(self, original_name, new_name):
        self.original_name  = original_name
        self.new_name       = new_name

    def sanitizeTopicName(self, topic_name):
        name = topic_name.replace("-", "_")
        return name

    def generateCode(self):
        text = ""

        # Aggiungo le informazioni alla remap
        text += "<remap from=\"{}\" to=\"{}\" />".format(self.sanitizeTopicName(self.original_name),
                                                         self.sanitizeTopicName(self.new_name))

        return text
