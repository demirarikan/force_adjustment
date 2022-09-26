#!/usr/bin/env python3
from matplotlib.animation import ImageMagickBase
import rospy
import imfusion
import numpy as np
import xml.etree.ElementTree as ET
import json
import math
import os
import xml.etree.ElementTree as ET
from matplotlib import pyplot as plt


imfusion.init() 

os.environ['PATH'] = 'C:\\Program Files\\ImFusion\\ImFusion Suite\\Suite;C:\\Program Files\\ImFusion' \
                     '\\ImFusion Suite\\Suite\\plugins;' + os.environ['PATH']


def cast_param(text):
    if text is None:
        return ""

    text_list = text.split(" ")

    if len(text_list) == 1:
        try:
            casted_value = float(text)
            return casted_value

        except:
            return text

    else:
        try:
            converted_list = [float(item) for item in text_list]
            return np.array(converted_list)
        except:
            return text


def get_block(parent_block, block_name):
    for child in parent_block:
        if child.attrib["name"] == block_name:
            return child


def parse_alg_to_dict(param_dict, block, prefix=""):
    for item in block:
        if item.tag == 'param':
            param_dict[prefix + item.attrib["name"]] = cast_param(item.text)
        elif item.tag == 'property':
            sub_block = get_block(block, item.attrib["name"])
            parse_alg_to_dict(param_dict, sub_block, prefix + item.attrib["name"] + "/")

    return param_dict


def read_roi_from_json(model_name: str):
    with open("imfusion_files/3dModels/" + model_name + "_ROI.json") as f:
        data = json.load(f)
        min_x_loc = data["min_x_loc"]
        max_x_loc = data["max_x_loc"]
        min_y_loc = data["min_y_loc"]
        max_y_loc = data["max_y_loc"]
        max_z_loc = data["max_z_loc"]
        min_z_loc = data["min_z_loc"]
        return min_x_loc, max_x_loc, min_y_loc, max_y_loc, min_z_loc, max_z_loc

def get_algorithm_properties_from_iws(iws_filepath, algorithm_name):
    """
    This function parse a iws file and saves all the parameters referred to the <algorithm_name> algorithm in a dict,
    that can be then used to create the algorithm properties.
    Every field of the dict can be overwritten with custom values. E.g. if one wants to overwrite Transducer and
    Direction spline in the dictionary obtained by passing algorithm_name='Hybrid Ultrasound Simulation', they can simply
    do:

    default_hybrid_simulation_params = get_algorithm_properties_from_iws(iws_filepath)
    default_hybrid_simulation_params[SplineDirection/points] = np.ndarray([...])
    default_hybrid_simulation_params[SplineTransducer/points] = np.ndarray([...])

    :param iws_filepath: The path to the iws file. It MUST contain the <algorithm_name> property!!
    :param algorithm_name: The algorithm for which you want to extract the parameters
    :return: A dictionary containing the <algorithm_name> parameters
    """

    tree = ET.parse(iws_filepath)
    root = tree.getroot()

    algorithms = get_block(root, "Algorithms")
    us_sim = get_block(algorithms, algorithm_name)

    p_dict = dict()
    parse_alg_to_dict(p_dict, us_sim)

    imfusion_properties = imfusion.Properties(p_dict)

    return imfusion_properties

    
print('BEFORE SIMULATED SWEEP \n')
simulated_sweep = imfusion.open('/home/demir/Desktop/shadowing-aware-robotic-ultrasound/sweeps/simulated_sweep_dims4106_pose0_0_0.imf')


        # Inverting sweep as confidence maps are computed bottom to top
flipped_sweeps = imfusion.executeAlgorithm('Basic Processing', simulated_sweep, imfusion.Properties( {'mode': 2,'flip': 1}))
print('FLIPPED SWEEPS \n')
image = flipped_sweeps[0]
print(image)
arr = np.array(image[0])
print(arr.shape)
confidence_maps_params = get_algorithm_properties_from_iws("/home/demir/thesis_catkin/src/force_adjustment/asdf.iws", 'Compute Confidence Maps')
print('Confidence map PARAM: \n', confidence_maps_params.params())
p = imfusion.algorithmProperties('Ultrasound;Compute Confidence Maps', flipped_sweeps)
print(p.params())
confidence_maps = imfusion.executeAlgorithm('Ultrasound;Compute Confidence Maps', flipped_sweeps, confidence_maps_params)
print(confidence_maps)
conf_map = confidence_maps[0]
arr = np.array(conf_map[0])
print(arr.shape)
plt.imshow(arr, interpolation='nearest')
plt.show()

print('CONFIDENCE MAPS DONE \n')
