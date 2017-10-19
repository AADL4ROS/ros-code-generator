import os
import shutil
import threads.AADLThreadFunctionsSupport as tfs

# Output
dir             = os.path.dirname(__file__)
output_folder   = os.path.join(dir, "src")

src_folder_name     = 'src'
service_folder_name = 'srv'
message_folder_name = 'msg'
launch_folder_name  = 'launch'
include_folder_name = 'include'

def createFolderTreeForSystem(namespace, delete = True):

    # Sanitize del nome della cartella del system
    #system_name = system_name.replace(".", "_")
    system_name = namespace.replace("/", "_")

    system_folder = os.path.join(output_folder, system_name)

    if delete:
        # Se la cartella già esiste, la rimuovo
        if os.path.exists(system_folder):
            shutil.rmtree(system_folder)

    # Creo la cartella base del system
    if not os.path.exists(system_folder):
        os.makedirs(system_folder)

    # Creo le altre cartelle
    other_folders = [src_folder_name, launch_folder_name, service_folder_name,
                     message_folder_name, include_folder_name]
    for f in other_folders:
        path = os.path.join(system_folder, f)
        if not os.path.exists(path):
            os.makedirs( path )

    # Per la cartella include creo anche la sottocartella con il nome del package
    path = os.path.join(system_folder, include_folder_name, system_name)
    if not os.path.exists(path):
        os.makedirs(path)

    return system_folder

def cleanLaunchFolderForSystemFolder(system_folder):
    output_folder = getLaunchFolderForSystemFolder(system_folder)
    for the_file in os.listdir(output_folder):
        file_path = os.path.join(output_folder, the_file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(e)

def getSrcFolderForSystemFolder(system_folder):
    return os.path.join(system_folder, src_folder_name)

def getLaunchFolderForSystemFolder(system_folder):
    return os.path.join(system_folder, launch_folder_name)

def getServiceFolderForSystemFolder(system_folder):
    return os.path.join(system_folder, service_folder_name)

def getMessageFolderForSystemFolder(system_folder):
    return os.path.join(system_folder, message_folder_name)

def getIncludeFolderForSystemFolder(system_folder):
    return os.path.join(system_folder, include_folder_name)

# Usato dal CMake per avere src/nome_file.cpp
def getOnlySrcPathForNode(node_filename):
    return os.path.join(src_folder_name, node_filename)