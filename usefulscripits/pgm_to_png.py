from PIL import Image

# Open the .pgm file
img = Image.open('../assets/mymap.pgm')

# Convert and save as .jpg
img.save('../assets/mymap.png')
