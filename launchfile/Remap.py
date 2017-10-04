import XMLTags
import logging
log = logging.getLogger("root")

class Remap():
    def __init__(self, original_name, new_name):
        self.original_name  = original_name
        self.new_name       = new_name

    def generateCode(self):
        text = ""

        # Aggiungo le informazioni alla remap
        text += "<remap from=\"{}\" to=\"{}\" />".format(self.original_name, self.new_name)

        return text
