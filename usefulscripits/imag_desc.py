# Import necessary libraries
from PIL import Image
import matplotlib.pyplot as plt
import yaml

path = r"/home/oscar/Desktop/SymbioticRobots/assets/"

import yaml
from PIL import Image

# Replace with your image file path
image_path = path+'full_map.png'
# Replace with your yaml file path
yaml_path = path+'full_map.yaml'

# Open and read the image
image = Image.open(image_path)
print('Image size:', image.size)

# Open and read the yaml file
with open(yaml_path, 'r') as stream:
    try:
        yaml_content = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
    else:
        print('YAML content:', yaml_content)



