import os
from lxml import etree

# Username and email of the main package maintainer
kMAINTAINER_NAME = "Name"
kMAINTAINER_EMAIL = "email@example.org"

kPACKAGE_FILENAME = "package.xml"


class PackageXML:
    def __init__(self, system):
        self.system = system
        self.project_name = system.namespace

        self.dependencies = []

        # Aggiungo i pacchetti standard
        self.addDependency("roscpp")

    def addDependency(self, lib):
        # Se sto aggiungendo una dependency tramite una stringa, allora non ho
        # bisogno di chiedere alla libreria il nome del package.
        if isinstance(lib, str):
            dep = lib
        else:
            dep = lib.getPackageName()

        # Non accade mai, ma nel caso non aggiungo me stesso
        # al file package XML come dipendenza. Ripeto, non accade
        # mai, è solo un controllo di sicurezza
        if lib == self.project_name:
            return

        self.dependencies.append(dep)

        # Rimuovo eventuali duplicati
        self.dependencies = list(set(self.dependencies))

    def removeDependency(self, lib):
        try:
            if isinstance(lib, str):
                dep = lib
            else:
                dep = lib.getPackageName()
            self.dependencies.remove(dep)
            return True
        except ValueError:
            return False

    def savePackageXML(self):
        output_folder = os.path.join(self.system.system_folder, kPACKAGE_FILENAME)
        with open(output_folder, 'wb+') as file:
            file.write(self.generateCode())

    def generateCode(self):
        # Creo l'elemento package che sarà la root
        package = etree.Element("package")

        # Name
        name = etree.Element("name")
        name.text = self.project_name
        package.append(name)

        # Version
        version = etree.Element("version")
        version.text = "0.0.0"
        package.append(version)

        # Description
        description = etree.Element("description")
        description.text = "The {} package".format(self.project_name)
        package.append(description)

        # Maintainer
        maintainer = etree.Element("maintainer")
        maintainer.text = kMAINTAINER_NAME
        maintainer.attrib["email"] = kMAINTAINER_EMAIL
        package.append(maintainer)

        # License
        license = etree.Element("license")
        license.text = "TODO"
        package.append(license)

        # buildtool_depend
        buildtool_depend = etree.Element("buildtool_depend")
        buildtool_depend.text = "catkin"
        package.append(buildtool_depend)

        # build_depend
        for d in self.dependencies:
            build_depend = etree.Element("build_depend")
            build_depend.text = d
            package.append(build_depend)

        # build_depend for messages and services
        build_depend = etree.Element("build_depend")
        build_depend.text = "message_generation"
        package.append(build_depend)

        # run_depend
        for d in self.dependencies:
            run_depend = etree.Element("run_depend")
            run_depend.text = d
            package.append(run_depend)

        # run_depend for messages and services
        run_depend = etree.Element("run_depend")
        run_depend.text = "message_runtime"
        package.append(run_depend)

        return etree.tostring(package,
                              pretty_print=True,
                              xml_declaration=True,
                              encoding="UTF-8")
