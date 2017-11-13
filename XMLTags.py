import json
import global_filepath
import os
#################
### PARAMETRI ###
#################

ocarina_ros_path    = global_filepath.xml_folder_path
json_filename       = global_filepath.json_filename

global tags

# Leggo il file JSON generato dal backend di Ocarina che contiene le mappature fra elementi e nomi
# nel file XML
with open(os.path.join(ocarina_ros_path, json_filename)) as data_file:
    tags = json.load( data_file )

# Stampo a schermo i tag per controllo
for tag in tags:
    print("{}: {}".format(tag, tags[tag]))