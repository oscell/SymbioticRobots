from PIL import Image

# Open the .pgm file
img = Image.open('C:\Users\OEM\Documents\GitHub\SymbioticRobots\assets\mymap.pgm')

# Convert and save as .jpg
img.save('output.jpg')
