from CObject import CObject

class Comment(CObject):
    def __init__(self, _associated_class = None):
        super().__init__(_associated_class)
        self.text = None

    def setComment(self, _text):
        self.text = _text

    def generateCode(self):
        if self.text == None:
            return ""

        split_string = self.text.split("\n")
        comment = "/**\n"

        for s in split_string:
            comment += " * " + s + "\n"

        comment += " */\n"
        return comment
