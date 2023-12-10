import yaml
import xml.etree.ElementTree as ET

# Load the radii.yaml file
with open('radii.yaml', 'r') as f:
    radii = yaml.safe_load(f)

# Parse the Gazebo world file
tree = ET.parse('forest.world')
root = tree.getroot()

# Initialize the obstacle counter
obstacle_id = 0

# Initialize the obstacles dictionary
obstacles = {}

# Loop through all the <model> tags
for model in root.findall('.//model'):
    if "Tree" in model.get('name') or "Bush" in model.get('name'):

        # Get the model name and radius from radii.yaml
        model_name = model.find('./link/collision/mesh/uri')
        radius = radii.get(model_name)

        # Skip this model if it doesn't have a radius in radii.yaml
        if not radius:
            continue

        # Get the model pose
        pose = model.find('pose').text.split()

        # Get the X and Y positions and convert them to floats
        x = float(pose[0])
        y = float(pose[1])

        # Add this obstacle to the obstacles dictionary
        obstacles[obstacle_id] = {'x': x, 'y': y, 'r': radius}

        # Increment the obstacle ID counter
        obstacle_id += 1

# Save the obstacles dictionary to map.yaml
with open('map.yaml', 'w') as f:
    yaml.safe_dump({'obstacles': obstacles}, f)
