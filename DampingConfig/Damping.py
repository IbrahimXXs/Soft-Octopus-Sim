import re

def add_damping_to_joints(xml_text, initial_damping=0.25, decay=0.92):
    # Find all joint tags with stiffness attribute
    pattern = r'(<joint[^>]*stiffness="[^"]*")([^>]*)(/?>)'
    joints = list(re.finditer(pattern, xml_text))
    
    if not joints:
        print("No joints with stiffness attribute found.")
        return xml_text

    modified_xml = xml_text
    offset = 0  # to keep track of added characters shifting positions
    damping = initial_damping

    for joint in joints:
        full_match = joint.group(0)
        before = joint.group(1)
        after = joint.group(2)
        end = joint.group(3)

        # Format damping to 6 decimal places
        damping_str = f' damping="{damping:.6f}"'

        # Construct the new joint tag
        new_joint = before + damping_str + after + end

        # Replace in the modified XML at the correct position
        start_index = joint.start() + offset
        end_index = joint.end() + offset
        modified_xml = modified_xml[:start_index] + new_joint + modified_xml[end_index:]

        # Update offset and damping for the next joint
        offset += len(new_joint) - len(full_match)
        damping *= decay

    return modified_xml

# === Main section ===
if __name__ == "__main__":
    # Replace this with your input file path
    input_path = "DampingConfig/XML_Editor.xml"
    output_path = "DampingConfig/updated_output_file.xml"  # or overwrite: input_path

    try:
        with open(input_path, "r", encoding="utf-8") as file:
            xml_content = file.read()

        updated_content = add_damping_to_joints(xml_content)

        with open(output_path, "w", encoding="utf-8") as file:
            file.write(updated_content)

        print(f"Updated XML written to: {output_path}")

    except FileNotFoundError:
        print(f"Error: File not found at path: {input_path}")
    except Exception as e:
        print(f"An error occurred: {e}")
