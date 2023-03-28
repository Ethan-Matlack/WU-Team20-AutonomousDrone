# Importing cv2 in Pycharm is slightly broken. Need to add the cv2 folder to the interpreter paths.
# Issue discussed here: https://youtrack.jetbrains.com/issue/PY-35691/Code-completion-doesnt-work-for-cv2-module
# Resolution can be found here: https://github.com/opencv/opencv/issues/20997#issuecomment-1328068006
# Using an older version also works (opencv-python==4.5.5.62)
import tkinter

import cv2 as cv
import numpy as np
import tkinter as ttk
import mss
import time


class GFG:
    def __init__(self, master=None):
        self.master = master

        # Initialized (x, y) velocities
        self.x_vel = 0
        self.y_vel = 0

        # Initialized canvas size
        canvas_width = 1000
        canvas_height = 500

        self.canvas = ttk.Canvas(master, width=canvas_width, height=canvas_height)

        # Initialized start position of ID square and their size
        initial_x = canvas_width / 2
        initial_y = canvas_height / 2
        square_size = 75

        self.rect_cyan = self.canvas.create_rectangle(
            initial_x - square_size, initial_y,
            initial_x, initial_y + square_size,
            fill="#0095cd", outline="")
        self.rect_magenta = self.canvas.create_rectangle(
            initial_x - square_size, initial_y - square_size,
            initial_x, initial_y,
            fill="#a21f4e", outline="")
        self.rect_yellow = self.canvas.create_rectangle(
            initial_x, initial_y - square_size,
            initial_x + square_size, initial_y,
            fill="#ffd018", outline="")
        self.rect_black = self.canvas.create_rectangle(
            initial_x, initial_y,
            initial_x + square_size, initial_y + square_size,
            fill="#22262b", outline="")

        self.line_to_center = self.canvas.create_line(
            initial_x, initial_y,
            canvas_width / 2, canvas_height / 2,
            fill="#39FF14", width=5)

        self.canvas.pack()
        self.movement()

    def movement(self):
        # This is where the move() method is called
        # This moves the rectangle to x, y coordinates
        self.canvas.move(self.rect_cyan, self.x_vel, self.y_vel)
        self.canvas.move(self.rect_magenta, self.x_vel, self.y_vel)
        self.canvas.move(self.rect_yellow, self.x_vel, self.y_vel)
        self.canvas.move(self.rect_black, self.x_vel, self.y_vel)
        self.canvas.coords(self.line_to_center,
                           self.canvas.coords(self.line_to_center)[0] + self.x_vel,
                           self.canvas.coords(self.line_to_center)[1] + self.y_vel,
                           self.canvas.winfo_width() / 2,
                           self.canvas.winfo_height() / 2)

        # Check if the object has hit the bounds of the window. If so, invert direction.
        if (
            self.canvas.coords(self.rect_magenta)[0] <= 0 or
            self.canvas.coords(self.rect_black)[2] >= self.canvas.winfo_width()
        ):
            self.x_vel = self.x_vel * -1
        if (
            self.canvas.coords(self.rect_magenta)[1] <= 0 or
            self.canvas.coords(self.rect_black)[3] >= self.canvas.winfo_height()
        ):
            self.y_vel = self.y_vel * -1

        self.canvas.after(20, self.movement)

    # for motion in negative x direction
    def left(self, event):
        print(event.keysym)
        self.x_vel = -5
        # self.y_vel = 0

    # for motion in positive x direction
    def right(self, event):
        print(event.keysym)
        self.x_vel = 5
        # self.y_vel = 0

    # for motion in positive y direction
    def up(self, event):
        print(event.keysym)
        # self.x_vel = 0
        self.y_vel = -5

    # for motion in negative y direction
    def down(self, event):
        print(event.keysym)
        # self.x_vel = 0
        self.y_vel = 5

    # def clockwise(self, event):
    #     print(event.keysym)


if __name__ == "__main__":
    # object of class Tk, responsible for creating a tkinter toplevel window
    master = ttk.Tk()
    gfg = GFG(master)

    # This will bind arrow keys to the tkinter toplevel which will navigate the image or drawing
    master.bind("<KeyPress-Left>", lambda e: gfg.left(e))
    master.bind("<KeyPress-Right>", lambda e: gfg.right(e))
    master.bind("<KeyPress-Up>", lambda e: gfg.up(e))
    master.bind("<KeyPress-Down>", lambda e: gfg.down(e))

    # Infinite loop breaks only by interrupt
    ttk.mainloop()

    with mss.mss() as sct:
        # Part of the screen to capture
        monitor = {"top": master.winfo_x(), "left": master.winfo_y(),
                   "width": master.winfo_width(), "height": master.winfo_height()}

        while "Screen capturing":
            last_time = time.time()

            # Get raw pixels from the screen, save it to a Numpy array
            img = np.array(sct.grab(monitor))

            # Display the picture
            cv.imshow("OpenCV/Numpy normal", img)

            # Press "q" to quit
            if cv.waitKey(25) & 0xFF == ord("q"):
                cv.destroyAllWindows()
                break
