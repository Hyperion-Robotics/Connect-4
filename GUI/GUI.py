import tkinter as tk

class CenteredCanvasApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Connect 4")
        self.root.attributes("-fullscreen", True)
        # self.root.attributes("-type", "splash")
        self.root.geometry("800x480")
        self.root.configure(bg="#4d4343")
        

        # Center the window on the screen
        # self.center_window()

        # Create a canvas and place it in the middle of the window
        self.canvas = tk.Canvas(self.root, bg="orange", width=470, height=350)
        self.canvas.grid(row=1, column=1)

        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        print(f"Canvas size: {width}x{height} pixels")

        currentXY=(40,40)
        for j in range(6):
            for i in range(7):
                self.draw_circle(self.canvas,currentXY[0],currentXY[1],25)
                currentXY=currentXY[0]+65,currentXY[1]
                print(currentXY)
            currentXY=currentXY=40,currentXY[1]+55

        # self.draw_circle(self.canvas,540/2,350/2,30)

        # Configure the grid layout
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(2, weight=1)


    def draw_circle(self,canvas, x, y, radius):

        # Calculate the bounding box for the circle
        x0 = x - radius
        y0 = y - radius
        x1 = x + radius
        y1 = y + radius

        # Create the circle on the canvas
        canvas.create_oval(x0, y0, x1, y1, outline="black", fill="white")


if __name__ == "__main__":
    # Create the main window
    root = tk.Tk()

    # Create an instance of the app
    app = CenteredCanvasApp(root)

    # Start the main event loop
    root.mainloop()
