import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
from queue import Queue
from datetime import datetime
import json
import os
import time

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dual Motor Control System")
        self.root.geometry("1000x750")
        
        # Serial connection
        self.serial_port = None
        self.running = False
        
        # Motor positions and status
        self.motor_positions = {0: 0, 1: 0}
        self.motor_moving = {0: False, 1: False}
        self.motor_errors = {0: False, 1: False}
        self.status_registers = {0: 0x0000, 1: 0x0000}
        
        # Saved positions and sequences
        self.saved_positions = {}
        self.sequences = {}
        self.matrix_coord_labels = {}
        self.home_coord_label = None
        self.sequence_running = False
        self.load_positions()
        self.load_sequences()
        
        # Command queue for thread safety
        self.command_queue = Queue()
        
        self.setup_ui()
        
    def setup_ui(self):
        """Create the main UI"""
        # Style configuration
        style = ttk.Style()
        style.configure('Connected.TLabel', foreground='green')
        style.configure('Disconnected.TLabel', foreground='red')
        style.configure('Moving.TLabel', foreground='blue')
        style.configure('Error.TLabel', background='red', foreground='white')
        style.configure('Danger.TButton', foreground='red', font=('Arial', 10, 'bold'))
        
        # Main notebook
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # --- Tab Creation and Setup ---
        # Create all tab frames first
        status_tab = ttk.Frame(notebook)
        control_tab = ttk.Frame(notebook)
        position_tab = ttk.Frame(notebook)
        sequence_tab = ttk.Frame(notebook)
        debug_tab = ttk.Frame(notebook)
        
        # Add tabs to the notebook in the desired display order
        notebook.add(status_tab, text="Connection & Status")
        notebook.add(control_tab, text="Manual Control")
        notebook.add(position_tab, text="Position Management")
        notebook.add(sequence_tab, text="Sequences")
        notebook.add(debug_tab, text="Debug Console")

        # Now, setup the content for each tab.
        # IMPORTANT: The sequence tab must be set up before the position tab
        # because the position tab's refresh function calls a function that
        # relies on a widget created in the sequence tab, preventing a crash.
        self.setup_status_tab(status_tab)
        self.setup_control_tab(control_tab)
        self.setup_sequence_tab(sequence_tab)
        self.setup_position_tab(position_tab)
        self.setup_debug_tab(debug_tab)
        
    def setup_status_tab(self, parent):
        """Setup connection and status tab"""
        # Connection frame
        conn_frame = ttk.LabelFrame(parent, text="Serial Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Port selection
        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill=tk.X)
        
        ttk.Label(port_frame, text="Port:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(port_frame, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT, padx=5)
        ttk.Button(port_frame, text="Connect", command=self.connect).pack(side=tk.LEFT, padx=5)
        ttk.Button(port_frame, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT, padx=5)
        
        self.conn_status = ttk.Label(port_frame, text="DISCONNECTED", style='Disconnected.TLabel')
        self.conn_status.pack(side=tk.LEFT, padx=20)
        
        # Motor status frames
        status_container = ttk.Frame(parent)
        status_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Motor 0 Status
        motor0_frame = ttk.LabelFrame(status_container, text="Motor 0 Status", padding=10)
        motor0_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.motor0_pos_label = ttk.Label(motor0_frame, text="Position: 0", font=('Arial', 12))
        self.motor0_pos_label.pack(anchor=tk.W, pady=2)
        
        self.motor0_status_label = ttk.Label(motor0_frame, text="Status: IDLE")
        self.motor0_status_label.pack(anchor=tk.W, pady=2)
        
        self.motor0_error_label = ttk.Label(motor0_frame, text="Errors: None")
        self.motor0_error_label.pack(anchor=tk.W, pady=2)
        
        self.motor0_hex_label = ttk.Label(motor0_frame, text="Register: 0x0000", font=('Courier', 10))
        self.motor0_hex_label.pack(anchor=tk.W, pady=2)
        
        # Motor 0 Status bits
        self.motor0_bits_frame = ttk.Frame(motor0_frame)
        self.motor0_bits_frame.pack(fill=tk.X, pady=10)
        self.motor0_status_bits = self.create_status_bits(self.motor0_bits_frame)
        
        # Motor 1 Status
        motor1_frame = ttk.LabelFrame(status_container, text="Motor 1 Status", padding=10)
        motor1_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.motor1_pos_label = ttk.Label(motor1_frame, text="Position: 0", font=('Arial', 12))
        self.motor1_pos_label.pack(anchor=tk.W, pady=2)
        
        self.motor1_status_label = ttk.Label(motor1_frame, text="Status: IDLE")
        self.motor1_status_label.pack(anchor=tk.W, pady=2)
        
        self.motor1_error_label = ttk.Label(motor1_frame, text="Errors: None")
        self.motor1_error_label.pack(anchor=tk.W, pady=2)
        
        self.motor1_hex_label = ttk.Label(motor1_frame, text="Register: 0x0000", font=('Courier', 10))
        self.motor1_hex_label.pack(anchor=tk.W, pady=2)
        
        # Motor 1 Status bits
        self.motor1_bits_frame = ttk.Frame(motor1_frame)
        self.motor1_bits_frame.pack(fill=tk.X, pady=10)
        self.motor1_status_bits = self.create_status_bits(self.motor1_bits_frame)

        # Quick Positions Matrix Frame
        matrix_frame = ttk.LabelFrame(parent, text="Quick Positions", padding=10)
        matrix_frame.pack(fill=tk.X, padx=10, pady=10)

        matrix_container = ttk.Frame(matrix_frame)
        matrix_container.pack()

        # Create 4x6 matrix of buttons and labels
        for r in range(4):
            for c in range(6):
                pos_num = r * 6 + c + 1
                pos_name = f"P{pos_num}"
                
                cell_frame = ttk.Frame(matrix_container)
                cell_frame.grid(row=r, column=c, padx=3, pady=3, sticky='ew')

                btn = ttk.Button(cell_frame, text=pos_name,
                                 command=lambda name=pos_name: self.go_to_named_position(name))
                btn.pack()
                
                coord_label = ttk.Label(cell_frame, text="(N/A, N/A)", font=('Arial', 8))
                coord_label.pack()
                self.matrix_coord_labels[pos_name] = coord_label

        # Home button inside the matrix frame
        home_cell_frame = ttk.Frame(matrix_frame)
        home_cell_frame.pack(pady=5)
        ttk.Button(home_cell_frame, text="Home", command=lambda: self.go_to_named_position("Home")).pack()
        self.home_coord_label = ttk.Label(home_cell_frame, text="(N/A, N/A)", font=('Arial', 8))
        self.home_coord_label.pack()

        # Control buttons
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(control_frame, text="Read Status", command=self.read_status).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Clear Errors", command=self.clear_errors).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="STOP ALL", command=self.emergency_stop, 
                  style='Danger.TButton').pack(side=tk.LEFT, padx=20)
        
        self.update_matrix_labels()
        self.refresh_ports()

    def go_to_named_position(self, pos_name):
        """Move to a named saved position."""
        if pos_name in self.saved_positions:
            pos = self.saved_positions[pos_name]
            motor0_pos = pos['motor0']
            motor1_pos = pos['motor1']

            self.send_command(f"MOVE,{motor0_pos},{motor1_pos}")
            self.m0_target_var.set(motor0_pos)
            self.m1_target_var.set(motor1_pos)
            self.log_console(f"Moving to saved position '{pos_name}' ({motor0_pos}, {motor1_pos})", 'info')
        else:
            self.log_console(f"Position '{pos_name}' not found in saved positions.", 'error')
            messagebox.showwarning("Position Not Found", f"Saved position '{pos_name}' does not exist.")

    def update_matrix_labels(self):
        """Update the coordinate labels under the quick position buttons."""
        for pos_name, label in self.matrix_coord_labels.items():
            if pos_name in self.saved_positions:
                coords = self.saved_positions[pos_name]
                x = coords.get('motor0', 'N/A')
                y = coords.get('motor1', 'N/A')
                label.config(text=f"({x}, {y})")
            else:
                label.config(text="(N/A, N/A)")
        
        if self.home_coord_label:
            if "Home" in self.saved_positions:
                coords = self.saved_positions["Home"]
                x = coords.get('motor0', 'N/A')
                y = coords.get('motor1', 'N/A')
                self.home_coord_label.config(text=f"({x}, {y})")
            else:
                self.home_coord_label.config(text="(N/A, N/A)")

    def create_status_bits(self, parent):
        """Create status bit indicators"""
        bits = {}
        bit_names = ['HIZ', 'BUSY', 'SW_F', 'SW_EVN', 'DIR', 'MOT_STAT', 'CMD_ERR', 
                     'STALL_A', 'STALL_B', 'OCD', 'TH_WARN', 'UVLO', 'WRONG_CMD', 'NOTPERF_CMD']
        
        for i, name in enumerate(bit_names):
            if i % 7 == 0 and i > 0:
                row_frame = ttk.Frame(parent)
                row_frame.pack(fill=tk.X)
            elif i == 0:
                row_frame = ttk.Frame(parent)
                row_frame.pack(fill=tk.X)
            
            bit_frame = ttk.Frame(row_frame)
            bit_frame.pack(side=tk.LEFT, padx=2)
            
            canvas = tk.Canvas(bit_frame, width=12, height=12)
            canvas.pack()
            indicator = canvas.create_oval(2, 2, 10, 10, fill='gray', outline='black')
            
            ttk.Label(bit_frame, text=name, font=('Arial', 8)).pack()
            
            bits[name] = (canvas, indicator)
        
        return bits
    
    def update_status_bits(self, motor_id, status_reg):
        """Update status bit indicators"""
        bits = self.motor0_status_bits if motor_id == 0 else self.motor1_status_bits
        
        # Map status bits (adjust these based on your PowerStep01 header file)
        bit_map = {
            'HIZ': (status_reg >> 0) & 1,
            'BUSY': not ((status_reg >> 1) & 1),  # BUSY is active low
            'SW_F': (status_reg >> 2) & 1,
            'SW_EVN': (status_reg >> 3) & 1,
            'DIR': (status_reg >> 4) & 1,
            'CMD_ERR': (status_reg >> 7) & 1,
            'STALL_A': not ((status_reg >> 12) & 1),  # Active low
            'STALL_B': not ((status_reg >> 13) & 1),  # Active low
            'OCD': not ((status_reg >> 15) & 1),      # Active low
            'UVLO': not ((status_reg >> 9) & 1),      # Active low
        }
        
        for name, (canvas, indicator) in bits.items():
            if name in bit_map:
                color = 'red' if bit_map[name] else 'gray'
                if name == 'BUSY' and bit_map[name]:
                    color = 'green'  # BUSY is good when active
                canvas.itemconfig(indicator, fill=color)
    
    def setup_control_tab(self, parent):
        """Setup manual control tab"""
        # Jog control frame
        jog_frame = ttk.LabelFrame(parent, text="Manual Jog Control", padding=10)
        jog_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Motor 0 controls
        m0_frame = ttk.LabelFrame(jog_frame, text="Motor 0", padding=10)
        m0_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        step_frame0 = ttk.Frame(m0_frame)
        step_frame0.pack(fill=tk.X, pady=5)
        ttk.Label(step_frame0, text="Step Size:").pack(side=tk.LEFT, padx=5)
        self.m0_step_var = tk.IntVar(value=100)
        ttk.Spinbox(step_frame0, from_=1, to=5000, textvariable=self.m0_step_var, 
                   width=10).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(m0_frame, text="<<< Fast", 
                  command=lambda: self.jog_motor(0, -self.m0_step_var.get()*10)).pack(fill=tk.X, pady=2)
        ttk.Button(m0_frame, text="<< Backward", 
                  command=lambda: self.jog_motor(0, -self.m0_step_var.get())).pack(fill=tk.X, pady=2)
        ttk.Button(m0_frame, text=">> Forward", 
                  command=lambda: self.jog_motor(0, self.m0_step_var.get())).pack(fill=tk.X, pady=2)
        ttk.Button(m0_frame, text=">>> Fast", 
                  command=lambda: self.jog_motor(0, self.m0_step_var.get()*10)).pack(fill=tk.X, pady=2)
        
        # Motor 1 controls
        m1_frame = ttk.LabelFrame(jog_frame, text="Motor 1", padding=10)
        m1_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        step_frame1 = ttk.Frame(m1_frame)
        step_frame1.pack(fill=tk.X, pady=5)
        ttk.Label(step_frame1, text="Step Size:").pack(side=tk.LEFT, padx=5)
        self.m1_step_var = tk.IntVar(value=100)
        ttk.Spinbox(step_frame1, from_=1, to=5000, textvariable=self.m1_step_var, 
                   width=10).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(m1_frame, text="<<< Fast", 
                  command=lambda: self.jog_motor(1, -self.m1_step_var.get()*10)).pack(fill=tk.X, pady=2)
        ttk.Button(m1_frame, text="<< Backward", 
                  command=lambda: self.jog_motor(1, -self.m1_step_var.get())).pack(fill=tk.X, pady=2)
        ttk.Button(m1_frame, text=">> Forward", 
                  command=lambda: self.jog_motor(1, self.m1_step_var.get())).pack(fill=tk.X, pady=2)
        ttk.Button(m1_frame, text=">>> Fast", 
                  command=lambda: self.jog_motor(1, self.m1_step_var.get()*10)).pack(fill=tk.X, pady=2)
        
        # Direct position control
        pos_frame = ttk.LabelFrame(parent, text="Direct Position Control", padding=10)
        pos_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Motor 0 position
        m0_pos_frame = ttk.Frame(pos_frame)
        m0_pos_frame.pack(fill=tk.X, pady=5)
        ttk.Label(m0_pos_frame, text="Motor 0 Target:").pack(side=tk.LEFT, padx=5)
        self.m0_target_var = tk.IntVar(value=0)
        ttk.Entry(m0_pos_frame, textvariable=self.m0_target_var, width=10).pack(side=tk.LEFT, padx=5)
        
        # Motor 1 position
        m1_pos_frame = ttk.Frame(pos_frame)
        m1_pos_frame.pack(fill=tk.X, pady=5)
        ttk.Label(m1_pos_frame, text="Motor 1 Target:").pack(side=tk.LEFT, padx=5)
        self.m1_target_var = tk.IntVar(value=0)
        ttk.Entry(m1_pos_frame, textvariable=self.m1_target_var, width=10).pack(side=tk.LEFT, padx=5)
        
        button_frame = ttk.Frame(pos_frame)
        button_frame.pack(fill=tk.X, pady=10)
        ttk.Button(button_frame, text="Move to Position", 
                  command=self.move_to_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Home", 
                  command=self.home).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Reset Position", 
                  command=self.reset_position).pack(side=tk.LEFT, padx=5)
        
    def setup_position_tab(self, parent):
        """Setup position management tab"""
        # Save current position
        save_frame = ttk.LabelFrame(parent, text="Save Current Position", padding=10)
        save_frame.pack(fill=tk.X, padx=10, pady=10)
        
        input_frame = ttk.Frame(save_frame)
        input_frame.pack(fill=tk.X)
        
        ttk.Label(input_frame, text="Position Name:").pack(side=tk.LEFT, padx=5)
        self.pos_name_var = tk.StringVar()
        ttk.Entry(input_frame, textvariable=self.pos_name_var, width=20).pack(side=tk.LEFT, padx=5)
        ttk.Button(input_frame, text="Save Current", command=self.save_current_position).pack(side=tk.LEFT, padx=5)
        
        self.current_pos_label = ttk.Label(save_frame, text="Current: M0=0, M1=0", font=('Arial', 11))
        self.current_pos_label.pack(anchor=tk.W, pady=5)
        
        # Saved positions list
        list_frame = ttk.LabelFrame(parent, text="Saved Positions", padding=10)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Treeview for positions
        tree_frame = ttk.Frame(list_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(tree_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.position_tree = ttk.Treeview(tree_frame, columns=('Motor0', 'Motor1'), 
                                          yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.position_tree.yview)
        
        self.position_tree.heading('#0', text='Name')
        self.position_tree.heading('Motor0', text='Motor 0')
        self.position_tree.heading('Motor1', text='Motor 1')
        
        self.position_tree.column('#0', width=200)
        self.position_tree.column('Motor0', width=100)
        self.position_tree.column('Motor1', width=100)
        
        self.position_tree.pack(fill=tk.BOTH, expand=True)
        self.position_tree.bind('<Double-Button-1>', self.on_position_double_click)
        
        # Buttons
        button_frame = ttk.Frame(list_frame)
        button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(button_frame, text="Go To", command=self.go_to_saved_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Delete", command=self.delete_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Export", command=self.export_positions).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Import", command=self.import_positions).pack(side=tk.LEFT, padx=5)
        
        self.refresh_position_list()
        
    def setup_sequence_tab(self, parent):
        """Setup sequence management tab"""
        main_pane = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left side: Sequence List
        seq_list_frame = ttk.LabelFrame(main_pane, text="Sequences", padding=10)
        main_pane.add(seq_list_frame, weight=1)

        self.sequence_tree = ttk.Treeview(seq_list_frame, columns=('Steps',), show='tree headings')
        self.sequence_tree.heading('#0', text='Name')
        self.sequence_tree.heading('Steps', text='Steps')
        self.sequence_tree.pack(fill=tk.BOTH, expand=True)
        self.sequence_tree.bind('<<TreeviewSelect>>', self.on_sequence_select)

        seq_btn_frame = ttk.Frame(seq_list_frame)
        seq_btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(seq_btn_frame, text="Run", command=self.run_sequence).pack(side=tk.LEFT)
        ttk.Button(seq_btn_frame, text="Delete", command=self.delete_sequence).pack(side=tk.LEFT)

        # Right side: Sequence Editor
        editor_frame = ttk.LabelFrame(main_pane, text="Sequence Editor", padding=10)
        main_pane.add(editor_frame, weight=2)
        
        ttk.Label(editor_frame, text="Sequence Name:").pack(anchor=tk.W)
        self.seq_name_var = tk.StringVar()
        ttk.Entry(editor_frame, textvariable=self.seq_name_var).pack(fill=tk.X)

        steps_frame = ttk.LabelFrame(editor_frame, text="Steps", padding=5)
        steps_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.sequence_steps_tree = ttk.Treeview(steps_frame, columns=('Position', 'Delay'), show='headings')
        self.sequence_steps_tree.heading('Position', text='Position')
        self.sequence_steps_tree.heading('Delay', text='Delay (s)')
        self.sequence_steps_tree.column('Delay', width=80, anchor=tk.CENTER)
        self.sequence_steps_tree.pack(fill=tk.BOTH, expand=True)

        add_step_frame = ttk.Frame(editor_frame)
        add_step_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(add_step_frame, text="Position:").pack(side=tk.LEFT)
        self.pos_to_add_combo = ttk.Combobox(add_step_frame, state='readonly')
        self.pos_to_add_combo.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        ttk.Label(add_step_frame, text="Delay (s):").pack(side=tk.LEFT)
        self.delay_var = tk.DoubleVar(value=1.0)
        ttk.Spinbox(add_step_frame, from_=0, to=60, increment=0.1, textvariable=self.delay_var, width=6).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(add_step_frame, text="Add Step", command=self.add_step_to_sequence).pack(side=tk.LEFT)

        editor_btn_frame = ttk.Frame(editor_frame)
        editor_btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(editor_btn_frame, text="Remove Step", command=self.remove_step_from_sequence).pack(side=tk.LEFT)
        ttk.Button(editor_btn_frame, text="Save Sequence", command=self.save_new_sequence).pack(side=tk.RIGHT)
        
        self.refresh_sequence_list()
        self.update_pos_to_add_combo()
        
    def setup_debug_tab(self, parent):
        """Setup debug console tab"""
        # Command input
        cmd_frame = ttk.Frame(parent)
        cmd_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(cmd_frame, text="Command:").pack(side=tk.LEFT, padx=5)
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var, width=50)
        cmd_entry.pack(side=tk.LEFT, padx=5)
        cmd_entry.bind('<Return>', lambda e: self.send_custom_command())
        ttk.Button(cmd_frame, text="Send", command=self.send_custom_command).pack(side=tk.LEFT, padx=5)
        ttk.Button(cmd_frame, text="Clear", command=self.clear_console).pack(side=tk.LEFT, padx=5)
        
        # Console output
        console_frame = ttk.LabelFrame(parent, text="Communication Log", padding=5)
        console_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.console_text = tk.Text(console_frame, height=25, width=80, 
                                   bg='black', fg='green', font=('Courier', 10))
        self.console_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(console_frame, command=self.console_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.console_text.config(yscrollcommand=scrollbar.set)
        
        # Configure text tags for coloring
        self.console_text.tag_config('tx', foreground='cyan')
        self.console_text.tag_config('rx', foreground='yellow')
        self.console_text.tag_config('error', foreground='red')
        self.console_text.tag_config('info', foreground='white')
        
    def refresh_ports(self):
        """Refresh available COM ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
            
    def connect(self):
        """Connect to serial port"""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a COM port")
            return
        
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.1)
            self.running = True
            self.conn_status.config(text="CONNECTED", style='Connected.TLabel')
            self.log_console(f"Connected to {port}", 'info')
            
            # Start read thread
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
            # Start command processor thread
            self.cmd_thread = threading.Thread(target=self.process_commands, daemon=True)
            self.cmd_thread.start()
            
            # Initial status read
            self.root.after(500, self.read_status)
            
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.conn_status.config(text="DISCONNECTED", style='Disconnected.TLabel')
        self.log_console("Disconnected", 'info')
        
    def read_serial(self):
        """Read serial data in background thread"""
        buffer = ""
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open:
                    if self.serial_port.in_waiting:
                        data = self.serial_port.read(self.serial_port.in_waiting).decode('ascii', errors='ignore')
                        buffer += data
                        
                        # Process complete lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self.log_console(f"RX: {line}", 'rx')
                                self.parse_response(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    self.log_console(f"Read error: {str(e)}", 'error')
                    
    def process_commands(self):
        """Process command queue in background thread"""
        while self.running:
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get()
                    if self.serial_port and self.serial_port.is_open:
                        self.serial_port.write((command + '\n').encode())
                        self.log_console(f"TX: {command}", 'tx')
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    self.log_console(f"Send error: {str(e)}", 'error')
                    
    def send_command(self, command):
        """Queue command for sending"""
        self.command_queue.put(command)
        
    def parse_response(self, data):
        """Parse response from STM32"""
        try:
            parts = data.split(',')
            
            if parts[0] == "STATUS" and len(parts) >= 6:
                motor_id = int(parts[1])
                position = int(parts[2])
                is_moving = int(parts[3])
                has_error = int(parts[4])
                status_reg = int(parts[5], 16)
                
                # Update stored values
                self.motor_positions[motor_id] = position
                self.motor_moving[motor_id] = bool(is_moving)
                self.motor_errors[motor_id] = bool(has_error)
                self.status_registers[motor_id] = status_reg
                
                # Update UI
                self.update_motor_display(motor_id)
                
            elif parts[0] == "ERROR":
                motor_id = int(parts[1]) if len(parts) > 1 else 0
                error_type = parts[2] if len(parts) > 2 else "UNKNOWN"
                description = parts[3] if len(parts) > 3 else ""
                self.log_console(f"Motor {motor_id} Error: {error_type} - {description}", 'error')
                
        except Exception as e:
            pass  # Silently ignore parse errors
            
    def update_motor_display(self, motor_id):
        """Update motor status display"""
        position = self.motor_positions[motor_id]
        is_moving = self.motor_moving[motor_id]
        has_error = self.motor_errors[motor_id]
        status_reg = self.status_registers[motor_id]
        
        if motor_id == 0:
            self.motor0_pos_label.config(text=f"Position: {position}")
            self.motor0_status_label.config(text=f"Status: {'MOVING' if is_moving else 'IDLE'}")
            self.motor0_error_label.config(text=f"Errors: {'YES' if has_error else 'None'}")
            self.motor0_hex_label.config(text=f"Register: 0x{status_reg:04X}")
            if is_moving:
                self.motor0_status_label.config(style='Moving.TLabel')
            else:
                 self.motor0_status_label.config(style='TLabel') # Reset style
        else:
            self.motor1_pos_label.config(text=f"Position: {position}")
            self.motor1_status_label.config(text=f"Status: {'MOVING' if is_moving else 'IDLE'}")
            self.motor1_error_label.config(text=f"Errors: {'YES' if has_error else 'None'}")
            self.motor1_hex_label.config(text=f"Register: 0x{status_reg:04X}")
            if is_moving:
                self.motor1_status_label.config(style='Moving.TLabel')
            else:
                 self.motor1_status_label.config(style='TLabel') # Reset style
                
        # Update status bits
        self.update_status_bits(motor_id, status_reg)
        
        # Update current position label in position tab
        self.current_pos_label.config(text=f"Current: M0={self.motor_positions[0]}, M1={self.motor_positions[1]}")
        
    def log_console(self, message, tag='info'):
        """Log message to console"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.console_text.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        self.console_text.see(tk.END)
        
    def clear_console(self):
        """Clear console"""
        self.console_text.delete(1.0, tk.END)
        
    def send_custom_command(self):
        """Send custom command"""
        command = self.cmd_var.get().strip()
        if command:
            self.send_command(command)
            self.cmd_var.set("")
            
    def read_status(self):
        """Request status update"""
        self.send_command("STATUS")
        
    def clear_errors(self):
        """Clear error flags"""
        self.send_command("STATUS,CLEAR")
        
    def emergency_stop(self):
        """Emergency stop all motors"""
        self.sequence_running = False # Stop any running sequence
        self.send_command("STOP")
        self.log_console("EMERGENCY STOP ACTIVATED", 'error')
        
    def jog_motor(self, motor_id, steps):
        """Jog a single motor"""
        current_pos = self.motor_positions[motor_id]
        new_pos = current_pos + steps
        
        if motor_id == 0:
            self.send_command(f"MOVE,{new_pos},{self.motor_positions[1]}")
        else:
            self.send_command(f"MOVE,{self.motor_positions[0]},{new_pos}")
            
    def move_to_position(self):
        """Move to specified position"""
        m0_target = self.m0_target_var.get()
        m1_target = self.m1_target_var.get()
        self.send_command(f"MOVE,{m0_target},{m1_target}")
        
    def home(self):
        """Home motors (hardware home)"""
        self.send_command("HOME")
        self.m0_target_var.set(0)
        self.m1_target_var.set(0)
        
    def reset_position(self):
        """Reset position counters"""
        self.send_command("RESET")
        self.read_status()
        
    def save_current_position(self):
        """Save current position"""
        name = self.pos_name_var.get().strip()
        if not name:
            messagebox.showerror("Error", "Please enter a position name")
            return
            
        self.saved_positions[name] = {
            'motor0': self.motor_positions[0],
            'motor1': self.motor_positions[1]
        }
        self.save_positions()
        self.refresh_position_list()
        self.pos_name_var.set("")
        self.log_console(f"Saved position '{name}'", 'info')
        
    def refresh_position_list(self):
        """Refresh position list display"""
        for item in self.position_tree.get_children():
            self.position_tree.delete(item)
            
        sorted_positions = sorted(self.saved_positions.items())
        for name, pos in sorted_positions:
            self.position_tree.insert('', 'end', text=name, 
                                     values=(pos['motor0'], pos['motor1']))
        self.update_matrix_labels()
        self.update_pos_to_add_combo()
                                     
    def go_to_saved_position(self):
        """Go to selected saved position"""
        selection = self.position_tree.selection()
        if not selection:
            messagebox.showerror("Error", "Please select a position")
            return
            
        item = self.position_tree.item(selection[0])
        name = item['text']
        self.go_to_named_position(name)
        
    def on_position_double_click(self, event):
        """Handle double-click on position"""
        self.go_to_saved_position()
        
    def delete_position(self):
        """Delete selected position"""
        selection = self.position_tree.selection()
        if not selection:
            return
            
        item = self.position_tree.item(selection[0])
        name = item['text']
        
        if messagebox.askyesno("Confirm", f"Delete position '{name}'?"):
            if name in self.saved_positions:
                del self.saved_positions[name]
                self.save_positions()
                self.refresh_position_list()
            
    def load_positions(self):
        """Load saved positions from file"""
        if os.path.exists('motor_positions.json'):
            try:
                with open('motor_positions.json', 'r') as f:
                    self.saved_positions = json.load(f)
            except:
                self.saved_positions = {}
                
    def save_positions(self):
        """Save positions to file"""
        try:
            with open('motor_positions.json', 'w') as f:
                json.dump(self.saved_positions, f, indent=2)
        except Exception as e:
            self.log_console(f"Failed to save positions: {str(e)}", 'error')
            
    def export_positions(self):
        """Export positions to file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.saved_positions, f, indent=2)
                messagebox.showinfo("Success", "Positions exported successfully")
            except Exception as e:
                messagebox.showerror("Error", str(e))
                
    def import_positions(self):
        """Import positions from file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    imported = json.load(f)
                self.saved_positions.update(imported)
                self.save_positions()
                self.refresh_position_list()
                messagebox.showinfo("Success", "Positions imported successfully")
            except Exception as e:
                messagebox.showerror("Error", str(e))
    
    # --- Sequence Methods ---
    
    def load_sequences(self):
        """Load sequences from file"""
        if os.path.exists('sequences.json'):
            try:
                with open('sequences.json', 'r') as f:
                    self.sequences = json.load(f)
            except Exception as e:
                self.sequences = {}
                self.log_console(f"Could not load sequences: {e}", "error")

    def save_sequences(self):
        """Save sequences to file"""
        try:
            with open('sequences.json', 'w') as f:
                json.dump(self.sequences, f, indent=2)
        except Exception as e:
            self.log_console(f"Failed to save sequences: {str(e)}", 'error')
            
    def refresh_sequence_list(self):
        """Refresh sequence list display"""
        for item in self.sequence_tree.get_children():
            self.sequence_tree.delete(item)
        
        sorted_sequences = sorted(self.sequences.items())
        for name, steps in sorted_sequences:
            self.sequence_tree.insert('', 'end', text=name, values=(len(steps),))
    
    def on_sequence_select(self, event):
        """Handle sequence selection"""
        selection = self.sequence_tree.selection()
        if not selection:
            return
        
        seq_name = self.sequence_tree.item(selection[0])['text']
        self.seq_name_var.set(seq_name)
        
        # Clear and populate steps tree
        for item in self.sequence_steps_tree.get_children():
            self.sequence_steps_tree.delete(item)
            
        for step in self.sequences.get(seq_name, []):
            self.sequence_steps_tree.insert('', 'end', values=(step['pos_name'], step['delay']))
    
    def update_pos_to_add_combo(self):
        """Update the combobox with saved positions"""
        pos_names = sorted(self.saved_positions.keys())
        self.pos_to_add_combo['values'] = pos_names
        if pos_names:
            self.pos_to_add_combo.current(0)
    
    def add_step_to_sequence(self):
        """Add a step to the editor tree"""
        pos_name = self.pos_to_add_combo.get()
        delay = self.delay_var.get()
        if not pos_name:
            messagebox.showwarning("Warning", "Please select a position.")
            return
        self.sequence_steps_tree.insert('', 'end', values=(pos_name, delay))
        
    def remove_step_from_sequence(self):
        """Remove a selected step from the editor tree"""
        selection = self.sequence_steps_tree.selection()
        if selection:
            self.sequence_steps_tree.delete(selection[0])
    
    def save_new_sequence(self):
        """Save or update a sequence"""
        name = self.seq_name_var.get().strip()
        if not name:
            messagebox.showerror("Error", "Please enter a sequence name.")
            return
            
        steps = []
        for item in self.sequence_steps_tree.get_children():
            values = self.sequence_steps_tree.item(item, 'values')
            steps.append({'pos_name': values[0], 'delay': float(values[1])})
            
        self.sequences[name] = steps
        self.save_sequences()
        self.refresh_sequence_list()
        self.log_console(f"Sequence '{name}' saved.", 'info')

    def delete_sequence(self):
        """Delete a selected sequence"""
        selection = self.sequence_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a sequence to delete.")
            return
            
        name = self.sequence_tree.item(selection[0])['text']
        if messagebox.askyesno("Confirm", f"Delete sequence '{name}'?"):
            if name in self.sequences:
                del self.sequences[name]
                self.save_sequences()
                self.refresh_sequence_list()
                self.log_console(f"Sequence '{name}' deleted.", 'info')

    def run_sequence(self):
        """Run the selected sequence"""
        if self.sequence_running:
            messagebox.showwarning("Busy", "A sequence is already running.")
            return
            
        selection = self.sequence_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a sequence to run.")
            return

        name = self.sequence_tree.item(selection[0])['text']
        steps = self.sequences.get(name, [])
        
        if not steps:
            messagebox.showwarning("Warning", "Sequence has no steps.")
            return
        
        self.sequence_running = True
        thread = threading.Thread(target=self._sequence_worker, args=(name, steps), daemon=True)
        thread.start()
        
    def _sequence_worker(self, name, steps):
        """Background worker to execute a sequence"""
        self.log_console(f"Starting sequence '{name}'...", 'info')
        
        for i, step in enumerate(steps):
            if not self.sequence_running:
                self.log_console("Sequence stopped by user.", "error")
                break
                
            pos_name = step['pos_name']
            delay = step['delay']
            
            self.log_console(f"Step {i+1}/{len(steps)}: Moving to '{pos_name}'", 'info')
            self.go_to_named_position(pos_name)
            
            # Wait for move to complete
            time.sleep(0.1) # small initial delay
            while self.motor_moving[0] or self.motor_moving[1]:
                if not self.sequence_running: break
                self.read_status()
                time.sleep(0.2)
            
            if not self.sequence_running:
                self.log_console("Sequence stopped by user.", "error")
                break
                
            self.log_console(f"Waiting for {delay} seconds...", 'info')
            time.sleep(delay)

        self.log_console(f"Sequence '{name}' finished.", 'info')
        self.sequence_running = False

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()