from PIL import Image
import os

from PIL import Image
import os

from PIL import Image
import os

# Get user input for the image name
image_name = raw_input("Enter the name of the image (without extension): ")

# Create the input file path
input_image_path = '{}.pgm'.format(image_name)

# Split the input file path into directory, base, and extension
dirname, basename = os.path.split(input_image_path)
name, ext = os.path.splitext(basename)

# Create output file path
output_image_path = os.path.join(dirname, '{}.png'.format(name))

# Open and convert the image
image = Image.open(input_image_path)
image.save(output_image_path, "PNG")

print("Converted '{}' to '{}'".format(input_image_path, output_image_path))

