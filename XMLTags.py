import json

#################
### PARAMETRI ###
#################

ocarina_ros_path    = "../ocarina-ros/"
json_filename       = "tag_ever_xml.json"

global tags

# Leggo il file JSON generato dal backend di Ocarina che contiene le mappature fra elementi e nomi
# nel file XML
with open(ocarina_ros_path + json_filename) as data_file:
    tags = json.load( data_file )

# Stampo a schermo i tag per controllo
for tag in tags:
    print("{}: {}".format(tag, tags[tag]))