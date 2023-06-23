from PIL import Image

# Open the .pgm file
img = Image.open('../assets/mymap1.pgm')

# Convert and save as .jpg
img.save('../assets/mymap1.png')
