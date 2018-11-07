import global_filepath

global_filepath.output_dir = "./"
global_filepath.xml_folder_path = "../ocarina-ros/"
global_filepath.aadl_model_dir = "../Osate/aadl-ros-new/packages/"

global_filepath.xml_filename = "container.impl_ever_xml.xml"

from code_generator import startGeneration

startGeneration()
