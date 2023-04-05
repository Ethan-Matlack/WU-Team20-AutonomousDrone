import cv2 as cv
import numpy as np

color_explore = np.zeros((300, 300, 3), np.uint8)
color_selected = np.zeros((300, 300, 3), np.uint8)


def write_to_file(R, G, B):
    f = open("saved_color.txt", "a")
    RGB_color = str(R) + "," + str(G) + "," + str(B) + str("\n")
    f.write(RGB_color)
    f.close()


def show_color(event, x, y, flags, param):
    B = img[y, x][0]
    G = img[y, x][1]
    R = img[y, x][2]
    color_explore[:] = (B, G, R)

    if event == cv.EVENT_LBUTTONDOWN:
        color_selected[:] = (B, G, R)

    if event == cv.EVENT_RBUTTONDOWN:
        B = color_selected[10, 10][0]
        G = color_selected[10, 10][1]
        R = color_selected[10, 10][2]
        print(R, G, B)
        write_to_file(R, G, B)
        print(hex(R), hex(G), hex(B))


cv.namedWindow('Input Image')

img_path = r"TrainingImages\real\indoor\IMG_6450.jpg"
img = cv.resize(cv.imread(img_path), (640, 480))

# mouse call back function declaration
cv.setMouseCallback('Input Image', show_color)

while True:

    cv.imshow('Input Image', img)
    cv.putText(color_explore, f"{color_explore[0][0]}", (25, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv.imshow('Color Picker', color_explore)

    # Press "k" to quit
    if cv.waitKey(27) == ord('k'):
        cv.destroyAllWindows()
        break
