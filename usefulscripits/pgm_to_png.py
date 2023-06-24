from PIL import Image

# Open the .pgm file
img = Image.open('../assets/full_map.pgm')

# Convert and save as .jpg
img.save('../assets/full_map.png')
