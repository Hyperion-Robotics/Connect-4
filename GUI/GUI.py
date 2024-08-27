import tkinter as tk
import threading
import serial
import time
import json

class Connect4:
    def __init__(self, root):
        self.root = root
        self.root.title("Connect 4")
        self.root.geometry("800x480")
        self.root.configure(bg="#4d4343")

        self.frame = tk.Frame(self.root, bg="#4d4343")
        self.frame.grid(row=0, column=0, sticky="nsew")

        # Configure the root grid to center the frame
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        self.board = [[0 for _ in range(6)] for l in range(7)]
        self.gui_board = [[0 for _ in range(6)] for l in range(7)]
        self.side = "purple"
        self.current_frame="main"
        self.settingName=[]
        self.settingStatus=[]
        self.after_ids=[]

        self.UpdateFromJSON()

        self.create_main_frame()


    def create_main_frame(self):
        self.clear_frame()
        self.current_frame="main"

        print(self.current_frame)

        # Create a canvas with a fixed size and place it in the second row, spanning 4 columns
        self.canvas = tk.Canvas(self.frame, bg="orange", width=470, height=350)
        self.canvas.grid(row=1, column=0, columnspan=4, padx=10, pady=0)

        # Create the label and 4 buttons and place them in the top row
        label = tk.Label(self.frame, text="Hyperion Robotics Connect-4", bg="#a6a2a2", fg="white", font=('Franklin Gothic Medium', 24),relief="raised",pady=10, width=35)
        button1 = tk.Button(self.frame, text="Play", command=self.create_play_menu, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
        button2 = tk.Button(self.frame, text="Settings", command=self.create_settings_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
 

        label.grid(row=0, column=1, columnspan=2, padx=10, pady=5, sticky="ew")
        button1.grid(row=2, column=1, padx=10, pady=5, sticky="ew")
        button2.grid(row=2, column=2, padx=10, pady=5, sticky="ew")
   

        self.draw_play_area(self.canvas)
        self.print_board(self.gui_board)
        self.animation(self.canvas, 0, 4)

        # Configure the grid layout within the frame to center content
        self.frame.grid_rowconfigure(0, weight=1)  # Row for the label
        self.frame.grid_rowconfigure(1, weight=1)  # Row for the canvas (fixed size, no expansion)
        self.frame.grid_rowconfigure(2, weight=1)  # Row for buttons
        self.frame.grid_columnconfigure(0, weight=1)  # Columns for buttons and canvas
        self.frame.grid_columnconfigure(1, weight=1)
        self.frame.grid_columnconfigure(2, weight=1)
        self.frame.grid_columnconfigure(3, weight=1)

    def create_play_menu(self):
        self.clear_frame()
        self.current_frame = "play_menu"

        # Create the buttons and arrange them vertically
        button1 = tk.Button(self.frame, text="New Game", command=self.create_game_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button2 = tk.Button(self.frame, text="Load Game", command=self.button_pressed, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 30), width=20,  height=3)
        button3 = tk.Button(self.frame, text="Back", command=self.create_main_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 30), width=20, height=3)

        # Place the buttons in separate rows without expanding to fill the row
        button1.grid(row=0, column=0, padx=10, pady=10)  # Removed sticky="ew"
        button2.grid(row=1, column=0, padx=10, pady=10)  # Removed sticky="ew"
        button3.grid(row=2, column=0, padx=10, pady=10)  # Removed sticky="ew"

        # Configure the grid layout within the frame to center content
        self.frame.grid_rowconfigure(0, weight=1)  # Row for the first button
        self.frame.grid_rowconfigure(1, weight=1)  # Row for the second button
        self.frame.grid_rowconfigure(2, weight=1)  # Row for the third button
        self.frame.grid_columnconfigure(0, weight=1)  # Single column for buttons

    def create_game_frame(self):
        self.clear_frame()
        self.current_frame="game"

        # Create a canvas with a fixed size and place it in the second row, spanning 4 columns
        self.canvas = tk.Canvas(self.frame, bg="orange", width=470, height=350)
        self.canvas.grid(row=1, column=0, columnspan=4, padx=10, pady=0)

        # Create the label and 4 buttons and place them in the top row
        button1 = tk.Button(self.frame, text="Save and Quit", command=self.create_play_menu, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
        button2 = tk.Button(self.frame, text="Quit", command=self.button_pressed, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
 

        button1.grid(row=2, column=1, padx=10, pady=5, sticky="ew")
        button2.grid(row=2, column=2, padx=10, pady=5, sticky="ew")
   

        self.draw_play_area(self.canvas)
        self.print_board(self.gui_board)
        # self.animation(self.canvas, 0, 4)

        # Configure the grid layout within the frame to center content
        self.frame.grid_rowconfigure(0, weight=1)  # Row for the label
        self.frame.grid_rowconfigure(1, weight=1)  # Row for the canvas (fixed size, no expansion)
        self.frame.grid_rowconfigure(2, weight=1)  # Row for buttons
        self.frame.grid_columnconfigure(0, weight=1)  # Columns for buttons and canvas
        self.frame.grid_columnconfigure(1, weight=1)
        self.frame.grid_columnconfigure(2, weight=1)
        self.frame.grid_columnconfigure(3, weight=1)

    def create_settings_frame(self):
        self.clear_frame()
        self.current_frame = "settings"

        # Create the buttons and arrange them vertically
        button1 = tk.Button(self.frame, text=self.settingName[0], command=lambda: self.toggleSetting(1), bg=self.settingStatus[0], fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button2 = tk.Button(self.frame, text=self.settingName[1], command=lambda: self.toggleSetting(2), bg=self.settingStatus[1], fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button3 = tk.Button(self.frame, text=self.settingName[2], command=lambda: self.toggleSetting(3), bg=self.side, fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        
        # Fixing the command functions for the buttons
        button4 = tk.Button(self.frame, text="Test IR Gates", command=self.button_pressed, bg="orange", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button5 = tk.Button(self.frame, text="Test Arm\nPositions", command=self.create_game_frame, bg="orange", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button6 = tk.Button(self.frame, text="Toggle\nPumps", command=self.button_pressed, bg="orange", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button7 = tk.Button(self.frame, text="Power", command=self.create_main_frame, bg="red", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button8 = tk.Button(self.frame, text="Back", command=self.create_main_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 30), width=20, height=3)

        # Place the buttons in separate rows without expanding to fill the row
        button1.grid(row=0, column=0, padx=10, pady=10)  
        button2.grid(row=1, column=0, padx=10, pady=10)  
        button3.grid(row=2, column=0, padx=10, pady=10)  
        button4.grid(row=3, column=0, padx=10, pady=10)  
        button5.grid(row=0, column=1, padx=10, pady=10)  
        button6.grid(row=1, column=1, padx=10, pady=10) 
        button7.grid(row=2, column=1, padx=10, pady=10)  
        button8.grid(row=3, column=1, padx=10, pady=10)   

        # Configure the grid layout within the frame to center content
        self.frame.grid_rowconfigure(0, weight=1)  # Row for the first button
        self.frame.grid_rowconfigure(1, weight=1)  # Row for the second button
        self.frame.grid_rowconfigure(2, weight=1)  # Row for the third button
        self.frame.grid_rowconfigure(3, weight=1)  # Row for the fourth button
        self.frame.grid_columnconfigure(0, weight=1)  # First column for buttons
        self.frame.grid_columnconfigure(1, weight=1)  # Second column for buttons

    #------------------------------------------------------------------ Back end functions-----------------------------------------------------------#

    def clear_frame(self):
        if self.current_frame=="main":
            self.cancel_all_animations()
        self.current_frame=""# Clears the active subframe to avoid updating deleted assets
        # Destroy all widgets in the sub_frame
        for widget in self.frame.winfo_children():
            widget.destroy()

            # Reset the row and column configurations
        for i in range(self.frame.grid_size()[1]):  # Number of rows
            self.frame.grid_rowconfigure(i, weight=0)

        for j in range(self.frame.grid_size()[0]):  # Number of columns
            self.frame.grid_columnconfigure(j, weight=0)

    def draw_play_area(self,canvas):
        currentXY=(40,40)
        for j in range(6):
            for i in range(7):
                self.gui_board[i][j]=currentXY
                self.draw_circle(canvas,currentXY[0],currentXY[1],25,"white")
                currentXY=currentXY[0]+65,currentXY[1]
                print(currentXY)
                
            currentXY=currentXY=40,currentXY[1]+55

    def restart_animation(self,canvas):
        currentXY=(40,40)
        for j in range(6):
            for i in range(7):
                self.gui_board[i][j]=currentXY
                self.draw_circle(canvas,currentXY[0],currentXY[1],25,"white")
                currentXY=currentXY[0]+65,currentXY[1]
                print(currentXY)
                
            currentXY=currentXY=40,currentXY[1]+55
        
        # canvas.after(200,self.animation(canvas,0,5))
        self.animation(canvas,0,4)

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

    def button_pressed(self):  #  For testing buttons
        print("button pressed")


    def toggleSetting(self,setting):

        print("toggling setting"+str(setting))

        if self.settingStatus[setting-1]=="green":
            self.settingStatus[setting-1]="red"
        elif self.settingStatus[setting-1]=="red":
            self.settingStatus[setting-1]="green"
        else:
            print("Error. The requested setting has unexpected value")

        if self.settingStatus[2]=="red":
            self.side="purple"
        elif self.settingStatus[2]=="green":
            self.side="blue"

        self.saveSettings()     #Saves the setting change to the settings JSON file 

        self.create_settings_frame()

    def UpdateFromJSON(self):
        #updates all the lists from the 2 json files
        with open(r"C:\Users\gkkti\Documents\Python\settings.json", "r") as json_file:
             settings = json.load(json_file)

        for item in settings:
            self.settingName.append(item["setting"])
            self.settingStatus.append(item["status"]) 

        # for side option that needs to be either purple or blue 

        if self.settingStatus[2]=="red":
            self.side="purple"
        elif self.settingStatus[2]=="green":
            self.side="blue"
   

    def saveSettings(self):
        #exports the settings.json file from the list values
        with open(r"C:\Users\gkkti\Documents\Python\settings.json", "r") as json_file:
             settings = json.load(json_file)

        i=0     
        for item in settings:
             item['status'] = self.settingStatus[i]
             i=i+1

        with open(r"C:\Users\gkkti\Documents\Python\settings.json", 'w') as file:
            json.dump(settings, file, indent=2)

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

    def animation(self, canvas, Y, X, color="purple"):
        print(self.current_frame)
        if self.current_frame != "main":
            return 0

        # Initial delay and setup
        delay = 125
        pucks_to_animate = 5 - Y

        # Define the sequence of animation steps
        for i in range(pucks_to_animate):
            current_row = 5 - i
            next_row = 5 - (i + 1)

            # Draw the current puck
            id1 = canvas.after(delay * (i * 2), lambda row=current_row: self.animate_puck(canvas, row, X, color))
            self.after_ids.append(id1)  # Store the ID

            # Erase the current puck (set to "white") and draw the next puck
            id2 = canvas.after(delay * (i * 2 + 1), lambda row=current_row, next_row=next_row: self.animate_puck(canvas, row, X, "white") or self.animate_puck(canvas, next_row, X, color))
            self.after_ids.append(id2)  # Store the ID

        # Recursive calls with tracking
        if Y == 0 and X == 4:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 0, 3, "blue"))
            self.after_ids.append(id)
        if Y == 0 and X == 3:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 1, 3, "purple"))
            self.after_ids.append(id)
        if Y == 1 and X == 3:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 0, 2, "blue"))
            self.after_ids.append(id)
        if Y == 0 and X == 2:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 1, 2, "purple"))
            self.after_ids.append(id)
        if Y == 1 and X == 2:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 2, 2, "blue"))
            self.after_ids.append(id)
        if Y == 2 and X == 2:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 2, 3, "purple"))
            self.after_ids.append(id)
        if Y == 2 and X == 3:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 1, 4, "blue"))
            self.after_ids.append(id)
        if Y == 1 and X == 4:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 0, 5, "purple"))
            self.after_ids.append(id)
        if Y == 0 and X == 5:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 2, 4, "blue"))
            self.after_ids.append(id)
        if Y == 2 and X == 4:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 0, 1, "purple"))
            self.after_ids.append(id)
        if Y == 0 and X == 1:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 1, 5, "blue"))
            self.after_ids.append(id)
        if Y == 1 and X == 5:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation(canvas, 3, 4, "purple"))
            self.after_ids.append(id)
        if Y == 3 and X == 4:
            id = canvas.after(delay * (pucks_to_animate * 2), lambda: self.animation_end(canvas, 1))
            self.after_ids.append(id)

    def animation_end(self, canvas, scene, flash_count=0):
        if self.current_frame != "main":
            return 0

        delay = 200
        max_flashes = 6  # 6 flashes correspond to 3 complete color changes

        # Determine the current color based on the flash count
        if flash_count < max_flashes:
            color = "yellow" if flash_count % 2 == 0 else "purple"

            # Flash the four pucks with the current color
            self.animate_puck(canvas, 0, 1, color)
            self.animate_puck(canvas, 1, 2, color)
            self.animate_puck(canvas, 2, 3, color)
            self.animate_puck(canvas, 3, 4, color)

            # Schedule the next flash
            id = canvas.after(delay, lambda: self.animation_end(canvas, scene, flash_count + 1))
            self.after_ids.append(id)
        else:
            # Ensure the last color is yellow
            self.animate_puck(canvas, 0, 1, "yellow")
            self.animate_puck(canvas, 1, 2, "yellow")
            self.animate_puck(canvas, 2, 3, "yellow")
            self.animate_puck(canvas, 3, 4, "yellow")

            print("Animation sequence completed.")
            # Call draw_play_area after 500 ms
            id = canvas.after(500, lambda: self.restart_animation(canvas))
            self.after_ids.append(id)

    def cancel_all_animations(self):
        # Cancel all scheduled `after` calls
        for after_id in self.after_ids:
            self.canvas.after_cancel(after_id)
        self.after_ids.clear()  # Clear the list after canceling

    def animate_puck(self,canvas,X,Y,puck_color="def"):
        if puck_color=="def":
            color=self.side
        else:
            color=puck_color
        # row,column=self.gui_board[(X-1)][5-(Y-1)] 
        row,column=self.gui_board[Y][5-X]

        print(row,column)
        self.draw_circle(canvas,row,column,25,color)


if __name__ == "__main__":
    # Create the main window
    root = tk.Tk()

    # Create an instance of the app
    app = Connect4(root)

    # Start the main event loop
    root.mainloop()
