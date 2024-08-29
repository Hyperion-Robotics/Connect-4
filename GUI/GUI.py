import numpy as np
import json
import math
import serial
import threading
import tkinter as tk
import algorithm as ai
from datetime import datetime
from tkinter import filedialog
import subprocess

from interbotix_xs_msgs.srv import TorqueEnable
import rclpy
from rclpy.node import Node

class torqueClient(Node):

    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(TorqueEnable, 'wx250/torque_enable')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TorqueEnable.Request()

    def send_request(self, cmd):
        if cmd==1:
            torque=True
        elif cmd==0:
            torque=False
        else:
            print("Error. Aborting")
            return 0
        self.req.cmd_type='group'
        self.req.name='all'
        self.req.enable=torque
        return self.cli.call_async(self.req)

class Interpiter:
    def __init__(self):
        pass

    @staticmethod
    def transpose_board(original_board):
        board = [[0 for _ in range(7)] for l in range(6)]
        for y in range(6):
            for x in range(7):
                # print(f"x:{x}, y:{y}")
                board[5-y][x] = original_board[x][y]
        return board


  
    
    @staticmethod
    def reverse_transpose_board(transposed_board):
        board = [[0 for _ in range(6)] for l in range(7)]
        for y in range(6):
            for x in range(7):
                board[x][5-y] = transposed_board[y][x]


        return board

    
    @staticmethod
    def convert_board(board, player_color):
        # Create a new board to store the converted values
        converted_board = [[0] * len(board[0]) for _ in range(len(board))]
        
        # Define the conversion mapping based on player_color
        if player_color == "purple":
            conversion = {'P': 1, 'B': 2}
        elif player_color == "blue":
            conversion = {'B': 1, 'P': 2}
        else:
            raise ValueError("Invalid player color. Must be 'purple' or 'blue'.")

        # Traverse the board and convert "P" and "B" based on the mapping
        for i in range(len(board)):
            for j in range(len(board[0])):
                if board[i][j] in conversion:
                    converted_board[i][j] = conversion[board[i][j]]
                else:
                    converted_board[i][j] = board[i][j]  # Keep 0s as they are

        return converted_board
    
    @staticmethod
    def reverse_convert_board(converted_board, player_color):
        # Create a new board to store the reconverted values
        original_board = [[0] * len(converted_board[0]) for _ in range(len(converted_board))]
        
        # Define the reverse conversion mapping based on player_color
        if player_color == "purple":
            reverse_conversion = {1: 'P', 2: 'B'}
        elif player_color == "blue":
            reverse_conversion = {1: 'B', 2: 'P'}
        else:
            raise ValueError("Invalid player color. Must be 'purple' or 'blue'.")

        # Traverse the board and reconvert 1 and 2 based on the mapping
        for i in range(len(converted_board)):
            for j in range(len(converted_board[0])):
                if converted_board[i][j] in reverse_conversion:
                    original_board[i][j] = reverse_conversion[converted_board[i][j]]
                else:
                    original_board[i][j] = converted_board[i][j]  # Keep 0s as they are

        return original_board
    
    @staticmethod
    def encrypt(board, player_color):
        # print(board)
        board=Interpiter.transpose_board(board)
        # print(board)
        board=Interpiter.convert_board(board, player_color)
        # print(board)
        board = np.array(board)

        print("NEW\n")
        print(board)

        return board
    
    @staticmethod
    def dencrypt(board, player_color):
        board = board.tolist()
        print(board)
        board=Interpiter.reverse_transpose_board(board)
        print(board)
        board=Interpiter.reverse_convert_board(board, player_color)
        print(board)
        

        return board



        




class Connect4:
    def __init__(self, root):
        rclpy.init()

        self.root = root
        self.root.title("Connect 4")
        self.root.geometry("800x480")
        self.root.configure(bg="#4d4343")
        self.root.attributes("-fullscreen", True)
        # self.root.attributes("-type", "splash")

        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

        self.arm_torque=torqueClient()

        Serial_thread = threading.Thread(target=self.read_Serial)

        self.frame = tk.Frame(self.root, bg="#4d4343")
        self.frame.grid(row=0, column=0, sticky="nsew")

        # Configure the root grid to center the frame
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        self.board = [[0 for _ in range(6)] for l in range(7)]
        self.gui_board = [[0 for _ in range(6)] for l in range(7)]
        self.side = "purple"
        self.turn = "player"
        self.current_frame="main"
        self.settingName=[]
        self.settingStatus=[]
        self.after_ids=[]
        self.armTorque=True
        self.magnetState=False

        self.UpdateFromJSON()

        self.create_main_frame()

        Serial_thread.start()


    def create_main_frame(self):
        self.clear_frame()
        self.current_frame="main"

        # Clear the board every time a game is done to avoid stacking pucks when old games are loaded
        # Could be removed after testing since normaly the create_main_function doesnt modify the self.board array
        # But for the testing since it does modify it for testing purposed it is required atm 
        self.board = [[0 for _ in range(6)] for l in range(7)] 

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
        # self.animation(self.canvas, 0, 4)



        self.draw_play_area(self.canvas)
        # self.print_board(self.gui_board)
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
        button1 = tk.Button(self.frame, text="New Game", command=self.create_new_game_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button2 = tk.Button(self.frame, text="Load Game", command=self.open_file_dialog, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 30), width=20,  height=3)
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

    def create_new_game_frame(self):
        self.timestamp = datetime.now().strftime('%d_%H_%M')
        self.board=[[0 for _ in range(6)] for l in range(7)]
        self.create_game_frame(self.board)

    def create_game_frame(self,board):
        self.clear_frame()
        self.current_frame="game"

        # Create a canvas with a fixed size and place it in the second row, spanning 4 columns
        self.canvas = tk.Canvas(self.frame, bg="orange", width=470, height=350)
        self.canvas.grid(row=1, column=0, columnspan=4, padx=10, pady=0)

        # Create the label and 4 buttons and place them in the top row
        button1 = tk.Button(self.frame, text="Save and Quit", command=self.create_main_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
        
        button1.grid(row=2, column=1, columnspan=2,padx=10, pady=5, sticky="ew")
   
        self.draw_play_area(self.canvas)
        # self.print_board(self.gui_board)
        self.draw_board_from_save(self.canvas,board)
        
        #All this here is for testing and the self.change_side function messes up the side because it swaps it every time the create_main_frame is called
        # self.add_puck_to_column(self.board,1)
        # self.change_sides()
        # self.add_puck_to_column(self.board,1)
        # self.change_sides()
        # self.add_puck_to_column(self.board,1)
        # self.change_sides()
        # self.add_puck_to_column(self.board,1)
        # self.change_sides()
        # self.add_puck_to_column(self.board,2)
        # self.change_sides()
        # self.add_puck_to_column(self.board,5)
        # self.print_board(self.board)
        # self.save_game(self.board)

        # Configure the grid layout within the frame to center content
        self.frame.grid_rowconfigure(0, weight=1)  # Row for the label
        self.frame.grid_rowconfigure(1, weight=1)  # Row for the canvas (fixed size, no expansion)
        self.frame.grid_rowconfigure(2, weight=1)  # Row for buttons
        self.frame.grid_columnconfigure(0, weight=1)  # Columns for buttons and canvas
        self.frame.grid_columnconfigure(1, weight=1)
        self.frame.grid_columnconfigure(2, weight=1)
        self.frame.grid_columnconfigure(3, weight=1)

    def create_confirm_loaded_game(self,board):
        self.clear_frame()
        self.current_frame="confirm_loaded_game"

        # Create a canvas with a fixed size and place it in the second row, spanning 4 columns
        self.canvas = tk.Canvas(self.frame, bg="orange", width=470, height=350)
        self.canvas.grid(row=1, column=0, columnspan=4, padx=10, pady=0)

        # Create the label and 4 buttons and place them in the top row
        label = tk.Label(self.frame, text="Is this correct", bg="#a6a2a2", fg="white", font=('Franklin Gothic Medium', 24),relief="raised",pady=10, width=35)
        button1 = tk.Button(self.frame, text="Confirm", command=lambda: self.create_game_frame(board), bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
        button2 = tk.Button(self.frame, text="Back", command=self.create_main_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
 
        label.grid(row=0, column=1,columnspan=2, padx=10, pady=5, sticky="ew")
        button1.grid(row=2, column=1, padx=10, pady=5, sticky="ew")
        button2.grid(row=2, column=2, padx=10, pady=5, sticky="ew")
   

        self.draw_play_area(self.canvas)
        # self.print_board(self.gui_board)
        self.draw_board_from_save(self.canvas,board)
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
        
        # Fixing the command functions for the buttons
        button3 = tk.Button(self.frame, text="Toggle Arm\nTorque", command= self.toggle_arm_torque, bg="orange", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button4 = tk.Button(self.frame, text="Test IR Gates", command=self.create_ir_test_menu, bg="orange", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button5 = tk.Button(self.frame, text="Test Arm\nPositions", command=self.create_arm_menu, bg="orange", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button6 = tk.Button(self.frame, text="Toggle\nPumps", command=self.toggle_pumps, bg="orange", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button7 = tk.Button(self.frame, text="Power", command=self.create_power_menu, bg="red", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
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

    def create_power_menu(self):
        self.clear_frame()
        self.current_frame = "power_menu"

        # Create the buttons and arrange them vertically
        button1 = tk.Button(self.frame, text="Reboot", command=self.reboot, bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20, height=3)
        button2 = tk.Button(self.frame, text="Shutdown", command=self.shutdown, bg="red", fg="white", font=('DejaVu Sans', 30), width=20,  height=3)
        button3 = tk.Button(self.frame, text="Quit", command=self.quit, bg="blue", fg="white", font=('DejaVu Sans', 30), width=20, height=3)
        button4 = tk.Button(self.frame, text="Back", command=self.create_main_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 30), width=20, height=3)

        # Place the buttons in separate rows without expanding to fill the row
        button1.grid(row=0, column=0, padx=10, pady=10)
        button2.grid(row=1, column=0, padx=10, pady=10)  
        button3.grid(row=0, column=1, padx=10, pady=10)  
        button4.grid(row=1, column=1, padx=10, pady=10)  

        # Configure the grid layout within the frame to center content
        self.frame.grid_rowconfigure(0, weight=1)  # Row for the first button
        self.frame.grid_rowconfigure(1, weight=1)  # Row for the second button
        self.frame.grid_columnconfigure(0, weight=1)  # Single column for buttons
        self.frame.grid_columnconfigure(1, weight=1)  # Single column for buttons

    def create_arm_menu(self):
        self.clear_frame()
        self.current_frame = "arm_menu"

        # Create the buttons and arrange them vertically
        button1 = tk.Button(self.frame, text="Home", command=lambda: self.move_arm(1), bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20, height=3)
        button2 = tk.Button(self.frame, text="Middle", command=lambda: self.move_arm(2), bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20,  height=3)
        button3 = tk.Button(self.frame, text="Gameboard\nnear", command=lambda: self.move_arm(3), bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20, height=3)
        button4 = tk.Button(self.frame, text="Gameboard\nfar", command=lambda: self.move_arm(4), bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20, height=3)
        button5 = tk.Button(self.frame, text="Puck Row\nLeft", command=lambda: self.move_arm(5), bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20, height=3)
        button6 = tk.Button(self.frame, text="Puck Row\nMiddle", command=lambda: self.move_arm(6), bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20,  height=3)
        button7 = tk.Button(self.frame, text="Puck Row\nRight", command=lambda: self.move_arm(7), bg="yellow", fg="black", font=('DejaVu Sans', 30), width=20, height=3)
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
        self.frame.grid_rowconfigure(2, weight=1)  # Row for the first button
        self.frame.grid_rowconfigure(3, weight=1)  # Row for the second button
        self.frame.grid_columnconfigure(0, weight=1)  # Single column for buttons
        self.frame.grid_columnconfigure(1, weight=1)  # Single column for buttons


    def create_ir_test_menu(self):
        self.clear_frame()
        self.current_frame="ir_menu"

        # Create a canvas with a fixed size and place it in the second row, spanning 4 columns
        self.canvas = tk.Canvas(self.frame, bg="orange", width=470, height=350)
        self.canvas.grid(row=1, column=0, columnspan=4, padx=10, pady=0)

        # Create the label and 4 buttons and place them in the top row
        label = tk.Label(self.frame, text="IR Gate Test", bg="#a6a2a2", fg="white", font=('Franklin Gothic Medium', 24),relief="raised",pady=10, width=35)
        button1 = tk.Button(self.frame, text="Back", command=self.create_main_frame, bg="#a6a2a2", fg="white", font=('DejaVu Sans', 18),width=20, pady=5)
 

        label.grid(row=0, column=1, padx=10, pady=5, sticky="ew")
        button1.grid(row=2, column=1, padx=10, pady=5, sticky="ew")
   

        self.draw_play_area(self.canvas)
        self.print_board(self.gui_board)

        # Configure the grid layout within the frame to center content
        self.frame.grid_rowconfigure(0, weight=1)  # Row for the label
        self.frame.grid_rowconfigure(1, weight=1)  # Row for the canvas (fixed size, no expansion)
        self.frame.grid_rowconfigure(2, weight=1)  # Row for buttons
        self.frame.grid_columnconfigure(0, weight=1)  # Columns for buttons and canvas
        self.frame.grid_columnconfigure(1, weight=1)
        self.frame.grid_columnconfigure(2, weight=1)

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
                # print(currentXY)
                
            currentXY=currentXY=40,currentXY[1]+55

    def restart_animation(self,canvas):
        currentXY=(40,40)
        for j in range(6):
            for i in range(7):
                self.gui_board[i][j]=currentXY
                self.draw_circle(canvas,currentXY[0],currentXY[1],25,"white")
                currentXY=currentXY[0]+65,currentXY[1]
                # print(currentXY)
                
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
        print(gate)
        for i in range(6):
            if gameboard[gate-1][i]==0:
                empty_row=i
                # print(i)
                break
        x=gate-1        
        y=empty_row
        if self.side=="purple":
            gameboard[x][y]="P"
        if self.side=="blue":
            gameboard[x][y]="B"
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

    def change_turn(self):
        if self.turn == "player":
            self.turn = "robot"
        elif self.turn == "robot":
            self.turn = "player"

    def button_pressed(self):  #  For testing buttons
        print("button pressed")

    def toggle_pumps(self):  #  For testing buttons
        print("Pumps toggled")
        cmd = "1,2,"

        self.ser.write(cmd.encode('utf-8'))

    def toggle_arm_torque(self):
        if self.armTorque:
            print("Disabling Arm Torque")
            self.armTorque=False
            self.arm_torque.send_request(0)
            
        elif not self.armTorque:
            print("Enablig Arm Torque")
            self.arm_torque.send_request(1)
            self.armTorque=True



    def move_arm(self, stance):
        print("Arm moved at pos "+str(stance))

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
        with open("/home/connect4/gui/settings.json", "r") as json_file:
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
        with open("/home/connect4/gui/settings.json", "r") as json_file:
             settings = json.load(json_file)

        i=0     
        for item in settings:
             item['status'] = self.settingStatus[i]
             i=i+1

        with open("/home/connect4/gui/settings.json", 'w') as file:
            json.dump(settings, file, indent=2)

    def open_file_dialog(self):
        # Open a file dialog that only allows selection of .txt files
        file_path = filedialog.askopenfilename(
            title="Select a Text File",
            filetypes=[("Text Files", "*.txt")]  # Only .txt files will be displayed
        )
        
        if file_path:
            print(f"Selected file: {file_path}")
            
            # Open the file in read mode and read its content
            with open(file_path, 'r') as file:
                content = file.read()
                print(f"File content:\n{content}")
                loaded_board=self.string_to_board(content)
                self.create_confirm_loaded_game(loaded_board)

    def save_game(self,gameboard):
        save=self.board_to_string(self.board)
        # for i in range(6):
        #     save=+gameboard[i]
        
        print(f"file to be saved: {save}")
        filename=self.timestamp
        with open(filename+".txt", "w") as file:
            file.write(save)

    def board_to_string(self, board):
        # Initialize an empty string to accumulate the result
        result = ''
        
        # Get the number of rows and columns
        num_rows = len(board)
        num_cols = len(board[0])
        
        # Iterate over columns (each index corresponds to the position in the row)
        for col in range(num_cols):
            # For each column, iterate over rows from bottom to top
            for row in (range(num_rows)):
                result += str(board[row][col])
        
        return result
    
    def string_to_board(self, string):
        # Get the number of rows and columns from the existing self.board
        num_rows = len(self.board)
        num_cols = len(self.board[0])
        
        # Initialize the board as an empty 2D list
        board = [[0 for _ in range(num_cols)] for _ in range(num_rows)]
        
        # Fill the board row by row
        index = 0
        for col in range(num_cols):
            for row in range(num_rows):
                board[row][col] = string[index]
                index += 1

        self.print_board(board)
                
        return board
    
    def draw_board_from_save(self,canvas,loaded_board):
        print(loaded_board)
        for i in range(7):
            for j in range(6):
                    if loaded_board[i][j]=="B":
                        self.draw_puck(canvas,j,i,"blue")
                    if loaded_board[i][j]=="P":
                        self.draw_puck(canvas,j,i,"purple")

    def shutdown(self):
        print("shuting down")
    
    def reboot(self):
        print("rebooting ")
    
    def quit(self):
        self.root.quit()  # Exits the Tkinter main loop
        self.root.destroy()  # Destroys the main window, freeing up resources


    def read_Serial(self):

        try:
            while True:
                # print("here")
                if self.ser.in_waiting > 0:  # Check if there is data waiting in the buffer
                    msg = self.ser.readline().decode('utf-8').rstrip()  # Read the data and decode it
                    print(msg)  # Print the data to the terminal
                    index,data=msg.split(",")
                    if index=="1":
                        print(data)
                        if self.current_frame=="game":
                            if self.turn == "player":
                                self.board=self.add_puck_to_column(self.board,int(data))
                                self.print_board(self.board)
                                board = Interpiter.encrypt(self.board, self.side)
                                column, minimax_score = ai.minimax(board, 5, -math.inf, math.inf, True)
                                self.board = Interpiter.dencrypt(board, self.side)
                                print(f"AI move {column}")
                            elif self.turn == "robot":
                                print(f"robot move {data}")
                                self.board=self.add_puck_to_column(self.board,int(data))
                                self.print_board(self.board)

                            self.change_sides()
                            self.change_turn()
                            # self.save_game(self.board)
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            self.ser.close()  # Make sure to close the port when done

    #------------------------------------------------------------------ Animation start-----------------------------------------------------------#

    def animation(self, canvas, Y, X, color="purple"):
        # print(self.current_frame)
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

        # print(row,column)
        self.draw_circle(canvas,row,column,25,color)


if __name__ == "__main__":
    # Create the main window
    root = tk.Tk()

    # Create an instance of the app
    app = Connect4(root)

    # Start the main event loop
    root.mainloop()
