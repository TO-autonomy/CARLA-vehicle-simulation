import xml.etree.ElementTree as ET
import json
import xmltodict


def parse_element_to_dict(element):
    elem_dict = {}

    if element.attrib:
        elem_dict.update(element.attrib)

    # Process child elements
    children = list(element)
    if children:
        # Create a dictionary for child elements
        child_dict = {}
        for child in children:
            child_name = child.tag
            child_data = parse_element_to_dict(child)
            if child_name == "origin":
                origin = child_data["origin"]
                origin_dict = {}
                child_data_xyz = origin["xyz"].split()
                for i, coord in enumerate(["x", "y", "z"]):
                    origin_dict[coord] = float(child_data_xyz[i])

                child_data_rpy = origin["rpy"].split()
                for i, coord in enumerate(["roll", "pitch", "yaw"]):
                    origin_dict[coord] = float(child_data_rpy[i])

                child_data = origin_dict
            
            if child_name not in child_dict:
                child_dict[child_name] = child_data
            else:
                # Handle multiple children with the same tag
                if isinstance(child_dict[child_name], list):
                    child_dict[child_name].append(child_data)
                else:
                    child_dict[child_name] = [child_dict[child_name], child_data]
        elem_dict.update(child_dict)
    else:
        # If there are no children, add the text content
        if element.text:
            elem_dict['text'] = element.text.strip()

    return {element.tag: elem_dict}


def parse_xml_to_json(xml_string):
    # Parse the XML string into an ElementTree object
    root = ET.fromstring(xml_string)

    # Parse the XML tree into a dictionary
    xml_dict = parse_element_to_dict(root)

    # Convert the dictionary to a JSON string
    json_data = json.dumps(xml_dict, indent=4)

    return json_data


def main_recursive():
    with open('nuscenes_car.urdf', 'r') as f:
        urdf_data = f.read()
    
    json_output = parse_xml_to_json(urdf_data)

    with open("nuscenes_car.json", "w") as json_file:
        json_file.write(json_output)


def main_simple(): # Not used currently
    with open("nuscenes_car.urdf") as xml_file:
        data_dict = xmltodict.parse(xml_file.read())
    print(data_dict)
    json_data = json.dumps(data_dict)
    with open("nuscenes_car.json", "w") as json_file:
        json_file.write(json_data)


if __name__ == "__main__":
    main_recursive()