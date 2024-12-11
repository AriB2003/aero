import pytesseract
import pathlib
from PIL import Image

pytesseract.pytesseract.tesseract_cmd
words = pathlib.Path('out2.txt')
values = []
temp = []
temp2 = []

# Load the image
image = Image.open("/Users/bradfordcooper/Documents/aero/ocrTest.png")

# Perform OCR
text = pytesseract.image_to_string(image,config='--psm 6')

#print(text)

#parse string
for char in text:

    #remove text
    if char.isdigit() or char in ['-', '.', ' ', ',']:
        temp.append(char)


for i in range(len(temp)):
    if temp[i] == ' ' and i + 1 < len(temp) and temp[i + 1] == '-':
        temp2.append(',')  # Replace space before '-'
    
    #skip non-digits after new lines
    elif temp[i] == '\n' and i + 1 < len(temp) and not temp[i + 1].isdigit():
        continue

    elif temp[i] == ' ' and i + 1 < len(temp) and (temp[i + 1] == '.' or temp[i + 1].isdigit()):
        continue  # Skip spaces before .'s or numbers
    else:
        temp2.append(temp[i])





print(temp2)


# with open("out2.txt", encoding="utf8") as file:
#     text = file.read()
#     for line in text.splitlines():


