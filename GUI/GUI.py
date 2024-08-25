import tkinter as tk
import threading
import serial
import time

class CenteredCanvasApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Connect 4")
        # self.root.attributes("-fullscreen", True)
        # self.root.attributes("-type", "splash")
        self.root.geometry("800x480")
        self.root.configure(bg="#4d4343")

        Serial_thread = threading.Thread(target=self.read_Serial)
        
        self.board=[[0 for _ in range(6)]for l in range(7)]
        self.gui_board=[[0 for _ in range(6)]for l in range(7)]
        self.side="purple"
        # self.print_board(self.board)

        # Serial_thread.setDaemon(True)
        # Serial_thread.start()



        # Create a canvas and place it in the middle of the window
        self.canvas = tk.Canvas(self.root, bg="orange", width=470, height=350)
        self.canvas.grid(row=1, column=1)

        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        print(f"Canvas size: {width}x{height} pixels")

        currentXY=(40,40)
        for j in range(6):
            for i in range(7):
                self.gui_board[i][j]=currentXY
                self.draw_circle(self.canvas,currentXY[0],currentXY[1],25,"white")
                currentXY=currentXY[0]+65,currentXY[1]
                print(currentXY)
                
            currentXY=currentXY=40,currentXY[1]+55

        self.print_board(self.gui_board)

        self.animate_puck(self.canvas,0,5)


        # self.draw_circle(self.canvas,540/2,350/2,30)

        # Configure the grid layout
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(2, weight=1)


    def draw_circle(self,canvas, x, y, radius,color):

        # Calculate the bounding box for the circle
        x0 = x - radius
        y0 = y - radius
        x1 = x + radius
        y1 = y + radius

        # Create the circle on the canvas
        canvas.create_oval(x0, y0, x1, y1, outline="black", fill=color)

    def print_board(self,gameboard):
        for i in range(6):
            for l in range(7):
                print(gameboard[l][5-i],end="")
            print("")
            
    def add_puck_to_column(self,gameboard,gate):
        empty_row=0
        for i in range(6):
            if gameboard[gate-1][i]==0:
                empty_row=i
                # print(i)
                break
        x=gate-1        
        y=empty_row
        gameboard[x][y]=1
        self.draw_puck(self.canvas,y,x)


        return gameboard
    
    def draw_puck(self,canvas,X,Y,puck_color="def"):
        if puck_color=="def":
            color=self.side
        else:
            color=puck_color
        # row,column=self.gui_board[(X-1)][5-(Y-1)] 
        row,column=self.gui_board[Y][5-X]

        print(row,column)
        self.draw_circle(canvas,row,column,25,color)



    def change_sides(self):
        if self.side=="purple":
            self.side="blue"
        elif self.side=="blue":
            self.side="purple"

    def read_Serial(self):
        ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

        try:
            while True:
                if ser.in_waiting > 0:  # Check if there is data waiting in the buffer
                    msg = ser.readline().decode('utf-8').rstrip()  # Read the data and decode it
                    print(msg)  # Print the data to the terminal
                    index,data=msg.split(",")
                    if index=="1":
                        self.board=self.add_puck_to_column(self.board,int(data))
                        self.print_board(self.board)
                        self.change_sides()
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            ser.close()  # Make sure to close the port when done

    #------------------------------------------------------------------ Animation start-----------------------------------------------------------#

    def animate_puck(self, canvas, Y, X,color="purple"):
        # Initial delay and setup
        delay = 350
        pucks_to_animate = 5 - Y
        
        # Define the sequence of animation steps
        for i in range(pucks_to_animate):
            current_row = 5 - i
            next_row = 5 - (i + 1)

            # Draw the current puck
            canvas.after(delay * (i * 2), lambda row=current_row: self.draw_puck(canvas, row, X,color))

            # Erase the current puck (set to "white") and draw the next puck
            canvas.after(delay * (i * 2 + 1), lambda row=current_row, next_row=next_row: self.draw_puck(canvas, row, X, "white") or self.draw_puck(canvas, next_row, X,color))

        # Final callback to return zero

        print(X,Y)
        if Y==0 and X==5: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,0,4,"blue"))
        if Y==0 and X==4: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,1,4,"purple"))
        if Y==1 and X==4: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,0,3,"blue"))
        if Y==0 and X==3: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,1,3,"purple"))
        if Y==1 and X==3: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,2,3,"blue"))
        if Y==2 and X==3: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,2,4,"purple"))
        if Y==2 and X==4: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,1,5,"blue"))
        if Y==1 and X==5: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,0,6,"purple"))
        if Y==0 and X==6: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,2,5,"blue"))
        if Y==2 and X==5: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,0,2,"purple"))
        if Y==0 and X==2: 
            canvas.after(delay * (pucks_to_animate * 2), lambda: self.animate_puck(canvas,0,2,"purple"))




if __name__ == "__main__":
    # Create the main window
    root = tk.Tk()

    # Create an instance of the app
    app = CenteredCanvasApp(root)

    # Start the main event loop
    root.mainloop()
