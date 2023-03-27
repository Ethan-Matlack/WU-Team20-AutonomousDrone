import cv2 as cv
import numpy as np

def main():
    print(f'')

img = np.zeros((512,512,3), np.uint8)
cv.rectangle(img,(384,0),(510,128),(0,255,0),3)
cv.imshow("Display", img)

if __name__ == '__main__':
    main()