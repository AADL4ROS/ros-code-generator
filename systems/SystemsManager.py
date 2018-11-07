# Vettore che contiene tutti i system
systems = []

# Vettore che contiene tutti i system già resettati, ovvero che i package
# che sono già stati eliminati. Per semplicità contiene solo il namespace
# del system
already_reset_systems = set()


def addSystem(system):
    global systems
    systems.append( system )


def addResetSystem(system_namespace):
    global already_reset_systems
    already_reset_systems.add(system_namespace)


def getSystemForNamespace(namespace):
    global systems
    for s in systems:
        if s.namespace == namespace:
            return s

    return None


def generateAllSystems():
    global systems
    for s in systems:
        s.generateSystem()


# Caso di system con altri system all'interno. Succede che magari alcuni di loro
# abbiano lo stesso namespace e quindi resettino di volta in volta i file generati.
# Se invece ne viene tenuta traccia in questa struttura, ogni namespace verrà resettato
# solamente una volta, senza perdere i file mano a mano generati
def isSystemAlreadyReset(system_namespace):
    global already_reset_systems
    return system_namespace in already_reset_systems
