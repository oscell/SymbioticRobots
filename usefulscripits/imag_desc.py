# Import necessary libraries
from PIL import Image
import matplotlib.pyplot as plt
import yaml

path = r"/home/oscar/Desktop/SymbioticRobots/assets/"


# Turn on interactive mode
plt.ion()

# Load the image
img = Image.open(path+'full_map.png')

# Display the image
plt.imshow(img)

# Draw and show the image for some time then close
plt.draw()
plt.pause(0.001)

# Output the shape of the image
width, height = img.size
print("The image shape is: {} pixels wide x {} pixels high".format(width, height))

# Load the YAML file
with open(path+'full_map.yaml', 'r') as file:
    yaml_file = yaml.safe_load(file)

# Print the contents of the YAML file
print(yaml.dump(yaml_file, default_flow_style=False))




# Close the plot
plt.close()
