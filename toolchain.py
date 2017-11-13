from optparse import OptionParser
import os
import sys
import re
from prototype_changer import replace_prototypes
from subprocess import Popen, PIPE
import shutil
import global_filepath

"""
Gli step di questa toolchain saranno:
1. Ricevere in input tutti i file necessari
2. Passare i file al prototype changer
3. Passare i file modificati ad ocarina (con possibilità di SSH)
4. Lanciare il code-generator
"""

def printHeader(text):
    text_len = len(text)
    out_str = ""
    out_str += "\n{}\n".format("#" * (text_len + 8))
    out_str += "{} {} {}\n".format("#" * 3, text, "#" * 3)
    out_str += "{}\n".format("#" * (text_len + 8))
    print(out_str)

# Get current folder
current_folder = os.getcwd()

# Get console argument
parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
                  help="The AADL file containing the model description", metavar="FILE")
parser.add_option("-s", "--system", dest="system",
                  help="The system to be generated (optional)")
parser.add_option("-o", "--output", dest="out_dir", metavar="DIR",
                  help="Output directory (optional)")
parser.add_option("-x", "--xml", dest="delete_xml_folder", default=True, action="store_false",
                  help="Delete XML model (optional, default = True)")

(options, args) = parser.parse_args()

if options.filename == None:
    print("Please provide an AADL file to process.")
    sys.exit(-1)

if options.out_dir == None:
    global_filepath.output_dir = current_folder
else:
    global_filepath.output_dir = os.path.realpath(os.path.join(current_folder, options.out_dir))

#########################################
### Step 1: Run the prototype changer ###
#########################################

printHeader("Starting procedure")

aadl_file_complete_path = os.path.join(current_folder, options.filename)

aadl_file_directory = os.path.dirname(aadl_file_complete_path)
aadl_file_name = os.path.basename(aadl_file_complete_path)

replace_prototypes(aadl_file_name, aadl_file_directory)

print("1. Prototype changed")

####################################
### Step 2: Collecting all files ###
####################################

# Leggo il contenuto della cartella che contiene il system che mi è stato passato con anche il suo file,
# a questo punto leggo tutti i with e per ogni file presente mi cerco se è da usare oppure no

file_package_mapping = {}
file_property_set_mapping = {}

find_package = re.compile('\s*package\s*(.+)')
find_with = re.compile('\s*with\s*(\w+\s*,?\s*)+\s*;')

find_property_set = re.compile('\s*property\s*set\s*(\w+)\s*is')

def getUsagePackages(file):
    matched_text = ""
    package_name = ""

    with open(file, 'r') as f:
        aadl_content = f.read()

    package_matches = find_package.findall(aadl_content)
    property_sets   = find_property_set.findall(aadl_content)

    for match in find_package.finditer(aadl_content):
        package_name = match.group(1)

    package_name = package_name.strip()

    for match in find_with.finditer(aadl_content):
        matched_text = match.group(0)

    matched_text = matched_text.replace("with", "").replace(";", "")
    matched_array = matched_text.split(",")
    matched_array = map(str.strip, matched_array)

    return (package_name, matched_array, property_sets)


for path, subdirs, files in os.walk(aadl_file_directory):
    for name in files:
        curr_file_path = os.path.realpath(os.path.join(path, name))

        (package, imported, property_sets) = getUsagePackages(curr_file_path)

        tmp_package = {'file'    : curr_file_path,
                       'imported': imported }

        if len(package) > 0:
            if package not in file_package_mapping:
                file_package_mapping[package] = tmp_package

            # Se ho il _mod nel file, sovrascrivo il vecchio valore
            elif "_mod.aadl" in name:
                file_package_mapping[package] = tmp_package

        for p_set in property_sets:
            if p_set not in file_property_set_mapping:
                file_property_set_mapping[p_set] = tmp_package

print("2. Files collected and analyzed")

(system_package, system_imported, _) = getUsagePackages(aadl_file_complete_path)

packages_to_import = [p for p in system_imported]

tmp_desc = file_package_mapping[system_package]
files_to_import = [ tmp_desc['file'] ]

def importFromDesc(tmp_desc):
    tmp_file_to_import = tmp_desc['file']
    if tmp_file_to_import not in files_to_import:
        files_to_import.append(tmp_file_to_import)
        packages_to_import.extend(tmp_desc['imported'])

# Importo via via che trovo i vari file che contengono i package usati
while len(packages_to_import) > 0:
    next_package = packages_to_import.pop()

    if next_package in file_package_mapping:
        tmp_desc = file_package_mapping[next_package]
        importFromDesc(tmp_desc)

    if next_package in file_property_set_mapping:
        tmp_desc = file_property_set_mapping[next_package]
        importFromDesc(tmp_desc)

print("3. The following files will be used in the process:")
for f in files_to_import:
    print("\t + " + f)

#######################
### Step 3: Ocarina ###
#######################

"""
ocarina -aadlv2 -g ever_xml [-r system_name] files
"""

xml_model_folder = os.path.join(global_filepath.output_dir, "generated_xml_model")

if os.path.exists(xml_model_folder):
    shutil.rmtree(xml_model_folder)

os.makedirs(xml_model_folder)

ocarina_parameters = ["ocarina", "-aadlv2", "-g", "ever_xml"]
# Compose paramters
if options.system != None:
    ocarina_parameters.append("-r")
    ocarina_parameters.append(options.system)

ocarina_parameters.extend(files_to_import)

p = Popen(ocarina_parameters, cwd=xml_model_folder, stdout=PIPE, stderr=PIPE)
output, err = p.communicate()
rc = p.returncode

if "Fine Ever XML" in output.decode():
    print("4. Ocarina backend ever_xml ended correctly.")
else:
    print(err.decode())
    print(output.decode())

###############################
### Step 4: Code Generation ###
###############################

# Set the global_filepath parameters
global_filepath.xml_folder_path = xml_model_folder
global_filepath.aadl_model_dir  = aadl_file_directory

from code_generator import startGeneration

for path, _, files in os.walk(xml_model_folder):
    for name in files:

        curr_file_path = os.path.realpath(os.path.join(path, name))

        (curr_file_name, curr_file_ext) = os.path.splitext(os.path.basename(name))

        if curr_file_ext.replace(".", "") == "xml":
            global_filepath.xml_filename = name
            startGeneration()

print("5. Code generation ended correctly.")

########################
### Step 4: Clean-up ###
########################

print("6. Cleaning up:")

if options.delete_xml_folder:
    if os.path.exists(xml_model_folder):
        try:
            shutil.rmtree(xml_model_folder)
            print("\t+ Removed XML model folder {}".format(xml_model_folder))
        except:
            print("\t+ Unable to remove XML model folder {}".format(xml_model_folder))
            pass
else:
    print("\t+ XML model folder available at {}".format(xml_model_folder))

for f in file_package_mapping:
    file_path = file_package_mapping[f]['file']

    if "_mod.aadl" in file_path:
        try:
            os.remove(file_path)
            print("\t+ Removed {}".format(file_path))
        except:
            print("\t+ Unable to remove {}".format(file_path))
            pass

printHeader("End procedure")
