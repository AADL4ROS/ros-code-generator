from optparse import OptionParser
import os
import sys
import re
from prototype_changer import replace_prototypes
from subprocess import Popen, PIPE
import shutil
import global_filepath
import log
from code_generator import startGeneration

"""
Gli step di questa toolchain saranno:
1. Ricevere in input tutti i file necessari
2. Passare i file al prototype changer
3. Passare i file modificati ad ocarina (con possibilità di SSH)
4. Lanciare il code-generator
"""


def print_header(text):
    text_len = len(text)
    out_str = ""
    out_str += "\n{}\n".format("#" * (text_len + 8))
    out_str += "{} {} {}\n".format("#" * 3, text, "#" * 3)
    out_str += "{}\n".format("#" * (text_len + 8))
    print(out_str)


def clean_up_aadl_modfile(aadl_file_directory, should_print=False):
    for path, _, files in os.walk(aadl_file_directory):
        for name in files:
            curr_file_path = os.path.realpath(os.path.join(path, name))

            if "_mod.aadl" in curr_file_path:
                try:
                    os.remove(curr_file_path)
                    if should_print:
                        print("\t+ Removed {}".format(curr_file_path))
                except OSError:
                    print("\t+ Unable to remove {}".format(curr_file_path))
                    pass


def get_usage_packages(file, find_package, find_property_set, find_with):
    matched_text = ""
    package_name = ""

    with open(file, 'r') as f:
        aadl_content = f.read()

    # package_matches = find_package.findall(aadl_content)
    property_sets = find_property_set.findall(aadl_content)

    for match in find_package.finditer(aadl_content):
        package_name = match.group(1)

    package_name = package_name.strip()

    for match in find_with.finditer(aadl_content):
        matched_text = match.group(0)

    matched_text = matched_text.replace("with", "").replace(";", "")
    matched_array = matched_text.split(",")
    matched_array = map(str.strip, matched_array)

    return package_name, matched_array, property_sets


def import_from_desc(tmp_desc, files_to_import, packages_to_import):
    tmp_file_to_import = tmp_desc['file']
    if tmp_file_to_import not in files_to_import:
        files_to_import.append(tmp_file_to_import)
        packages_to_import.extend(tmp_desc['imported'])


def main():
    # Get current folder
    current_folder = os.getcwd()
    step_number = 0

    # Get console argument
    parser = OptionParser()
    parser.add_option("-f", "--file", dest="filename",
                      help="The AADL file containing the model description (mandatory)", metavar="FILE")
    parser.add_option("-s", "--system", dest="system",
                      help="The system to be generated (optional)")
    parser.add_option("-o", "--output", dest="out_dir", metavar="DIR",
                      help="Output directory (optional)")
    parser.add_option("-x", "--xml", dest="delete_xml_folder", default=True, action="store_false",
                      help="keep the XML model (optional)")
    parser.add_option("-l", "--log", dest="log_level", metavar="LEVEL",
                      help="Log level. Valid values: [INFO, WARNING, ERROR]")

    (options, args) = parser.parse_args()

    if not options.filename:
        print("Please provide an AADL file to process.")
        sys.exit(-1)

    if not options.out_dir:
        global_filepath.output_dir = current_folder
    else:
        global_filepath.output_dir = os.path.realpath(os.path.join(current_folder, options.out_dir))

    # Step 1: Run the prototype changer

    print_header("Starting procedure")

    if options.log_level:
        level = str(options.log_level).upper()
        if not log.is_valid_log_level(level):
            print("Invalid log level {}. Used default INFO. See --help for more information.".format(level))
        else:
            log.set_log_level(level)
            print("Log level: {}".format(level))

    aadl_file_complete_path = os.path.join(current_folder, options.filename)

    aadl_file_directory = os.path.dirname(aadl_file_complete_path)
    aadl_file_name = os.path.basename(aadl_file_complete_path)

    step_number += 1
    print("{}. Environment initialization".format(step_number))

    clean_up_aadl_modfile(aadl_file_directory, True)

    replace_prototypes(aadl_file_name, aadl_file_directory)

    step_number += 1
    print("{}. Prototype changed".format(step_number))

    # Step 2: Collecting all files

    # Leggo il contenuto della cartella che contiene il system che mi è stato passato con anche il suo file,
    # a questo punto leggo tutti i with e per ogni file presente mi cerco se è da usare oppure no

    file_package_mapping = {}
    file_property_set_mapping = {}

    find_package = re.compile('\s*package\s*(.+)')
    find_with = re.compile('\s*with\s*(\w+\s*,?\s*)+\s*;')
    find_property_set = re.compile('\s*property\s*set\s*(\w+)\s*is')

    for path, subdirs, files in os.walk(aadl_file_directory):
        for name in files:

            # Skip per i file di sistema
            if name[0] == ".":
                continue

            curr_file_path = os.path.realpath(os.path.join(path, name))
            (package, imported, property_sets) = get_usage_packages(curr_file_path,
                                                                    find_package, find_property_set, find_with)

            tmp_package = {'file': curr_file_path,
                           'imported': imported}

            if len(package) > 0:
                if package not in file_package_mapping:
                    file_package_mapping[package] = tmp_package

                # Se ho il _mod nel file, sovrascrivo il vecchio valore
                elif "_mod.aadl" in name:
                    file_package_mapping[package] = tmp_package

            for p_set in property_sets:
                if p_set not in file_property_set_mapping:
                    file_property_set_mapping[p_set] = tmp_package

    step_number += 1
    print("{}. Files collected and analyzed".format(step_number))

    (system_package, system_imported, _) = get_usage_packages(aadl_file_complete_path,
                                                              find_package, find_property_set, find_with)

    packages_to_import = [p for p in system_imported]

    tmp_desc = file_package_mapping[system_package]
    files_to_import = [tmp_desc['file']]

    # Importo via via che trovo i vari file che contengono i package usati
    while len(packages_to_import) > 0:
        next_package = packages_to_import.pop()

        if next_package in file_package_mapping:
            tmp_desc = file_package_mapping[next_package]
            import_from_desc(tmp_desc, files_to_import, packages_to_import)

        if next_package in file_property_set_mapping:
            tmp_desc = file_property_set_mapping[next_package]
            import_from_desc(tmp_desc, files_to_import, packages_to_import)

    step_number += 1
    print("{}. The following files will be used in the process:".format(step_number))
    for f in files_to_import:
        print("\t + " + f)

    # Step 3: Ocarina

    """
    ocarina -aadlv2 -g ever_xml [-r system_name] files
    """

    xml_model_folder = os.path.join(global_filepath.output_dir, "generated_xml_model")

    if os.path.exists(xml_model_folder):
        shutil.rmtree(xml_model_folder)

    os.makedirs(xml_model_folder)

    ocarina_parameters = ["ocarina", "-aadlv2", "-g", "ever_xml"]
    # Compose paramters
    if options.system:
        ocarina_parameters.append("-r")
        ocarina_parameters.append(options.system)

    ocarina_parameters.extend(files_to_import)

    p = Popen(ocarina_parameters, cwd=xml_model_folder, stdout=PIPE, stderr=PIPE)
    output, err = p.communicate()
    # rc = p.returncode

    step_number += 1
    if "Fine Ever XML" in output.decode():
        print("{}. Ocarina backend ever_xml ended correctly.".format(step_number))
    else:
        print("{}. Error in Ocarina backend ever_xml".format(step_number))
        print("\nOcarina errors:\n")
        print(err.decode())

        print("\nOcarina output:\n")
        print(output.decode())
        sys.exit(0)

    # Step 4: Code Generation

    # Set the global_filepath parameters
    global_filepath.xml_folder_path = xml_model_folder
    global_filepath.aadl_model_dir = aadl_file_directory

    for path, _, files in os.walk(xml_model_folder):
        for name in files:

            # curr_file_path = os.path.realpath(os.path.join(path, name))

            (curr_file_name, curr_file_ext) = os.path.splitext(os.path.basename(name))

            if curr_file_ext.replace(".", "") == "xml":
                global_filepath.xml_filename = name
                startGeneration()
    step_number += 1
    print("{}. Code generation ended correctly.".format(step_number))

    # Step 4: Clean-up

    step_number += 1
    print("{}. Cleaning up:".format(step_number))

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

    clean_up_aadl_modfile(aadl_file_directory, should_print=True)

    print_header("End procedure")


if __name__ == "__main__":
    main()
