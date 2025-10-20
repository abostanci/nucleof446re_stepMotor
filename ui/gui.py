import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
from queue import Queue, Empty
from datetime import datetime
import json
import os
import time
import logging
from contextlib import contextmanager

# Configuration Constants
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1
MAX_POSITION = 999999
MIN_POSITION = -999999
# STATUS_POLL_INTERVAL = 1  # seconds
SEQUENCE_POLL_INTERVAL = 1
BUTTON_MATRIX_ROWS = 4
BUTTON_MATRIX_COLS = 6
POSITIONS_FILE = 'motor_positions.json'
SEQUENCES_FILE = 'sequences.json'

# Setup logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Step Motor Controller")
        self.root.geometry("1280x780")
        
        # Thread safety
        self.state_lock = threading.Lock()
        self.command_lock = threading.Lock()
        
        # Serial connection
        self.serial_port = None
        self.running = False
        
        # Motor state with thread-safe access
        self.motor_positions = {0: 0, 1: 0}
        self.motor_moving = {0: False, 1: False}
        self.motor_errors = {0: False, 1: False}
        self.status_registers = {0: 0x0000, 1: 0x0000}
        
        # Timing for rate-limiting
        self.last_status_request = 0
        self.auto_refresh_enabled = True  # Enable by default when connected
        self.status_poll_interval = 0.2  # seconds - changeable at runtime
        self.status_update_job = None  # Track scheduled status updates
        
        # Saved positions and sequences
        self.saved_positions = {}
        self.sequences = {}
        self.matrix_coord_labels = {}
        self.home_coord_label = None
        self.sequence_running = False
        self.stop_sequence_flag = threading.Event()
        
        # Command queue for thread safety
        self.command_queue = Queue()
        
        # Response queue for parsing
        self.response_queue = Queue()
        
        self.load_positions()
        self.load_sequences()
        self.setup_ui()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def _get_motor_state(self, motor_id):
        """Thread-safe getter for motor state"""
        with self.state_lock:
            return {
                'position': self.motor_positions.get(motor_id, 0),
                'moving': self.motor_moving.get(motor_id, False),
                'error': self.motor_errors.get(motor_id, False),
                'status_reg': self.status_registers.get(motor_id, 0x0000)
            }
    
    def _set_motor_state(self, motor_id, position=None, moving=None, error=None, status_reg=None):
        """Thread-safe setter for motor state"""
        with self.state_lock:
            if position is not None:
                self.motor_positions[motor_id] = position
            if moving is not None:
                self.motor_moving[motor_id] = moving
            if error is not None:
                self.motor_errors[motor_id] = error
            if status_reg is not None:
                self.status_registers[motor_id] = status_reg
    
    def setup_ui(self):
        """Create the main UI"""
        self._configure_styles()
        
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create tabs
        status_tab = ttk.Frame(notebook)
        control_tab = ttk.Frame(notebook)
        position_tab = ttk.Frame(notebook)
        sequence_tab = ttk.Frame(notebook)
        debug_tab = ttk.Frame(notebook)
        
        notebook.add(status_tab, text="Connection & Status")
        notebook.add(control_tab, text="Manual Control")
        notebook.add(position_tab, text="Position Management")
        notebook.add(sequence_tab, text="Sequences")
        notebook.add(debug_tab, text="Debug Console")
        
        self.setup_status_tab(status_tab)
        self.setup_control_tab(control_tab)
        self.setup_sequence_tab(sequence_tab)
        self.setup_position_tab(position_tab)
        self.setup_debug_tab(debug_tab)
        
        # Now that console exists, refresh ports
        self.refresh_ports()
        
    def _configure_styles(self):
        """Configure ttk styles"""
        style = ttk.Style()
        style.configure('Connected.TLabel', foreground='green', font=('Arial', 10, 'bold'))
        style.configure('Disconnected.TLabel', foreground='red', font=('Arial', 10, 'bold'))
        style.configure('Moving.TLabel', foreground='blue')
        style.configure('Error.TLabel', background='red', foreground='white')
        style.configure('Danger.TButton', foreground='red', font=('Arial', 10, 'bold'))

    def setup_status_tab(self, parent):
        """Setup connection and status tab"""
        self._create_connection_frame(parent)
        self._create_motor_status_frames(parent)
        self._create_quick_positions_matrix(parent)
        self._create_status_control_buttons(parent)
        
        self.update_matrix_labels()
        # Note: refresh_ports() called after UI setup complete

    def _create_connection_frame(self, parent):
        """Create serial connection control frame"""
        conn_frame = ttk.LabelFrame(parent, text="Serial Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=10)
        
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
        
        # Auto-refresh checkbox
        interval_frame = ttk.Frame(conn_frame)
        interval_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(interval_frame, text="Status Update Interval (ms):").pack(side=tk.LEFT, padx=5)
        self.status_interval_var = tk.IntVar(value=200)
        interval_spinbox = ttk.Spinbox(interval_frame, from_=50, to=5000, increment=50, 
                                       textvariable=self.status_interval_var, width=10)
        interval_spinbox.pack(side=tk.LEFT, padx=5)
        ttk.Button(interval_frame, text="Apply", command=self.apply_status_interval).pack(side=tk.LEFT, padx=5)
        ttk.Label(interval_frame, text="(lower = faster updates, higher = less traffic)").pack(side=tk.LEFT, padx=5)
        self.auto_refresh_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(port_frame, text="Auto-refresh", 
                       variable=self.auto_refresh_var,
                       command=self.toggle_auto_refresh).pack(side=tk.LEFT, padx=10)
        
    def apply_status_interval(self):
        """Apply status update interval"""
        interval_ms = self.status_interval_var.get()
        self.status_poll_interval = interval_ms / 1000.0  # Convert to seconds
        self.log_console(f"Status interval changed to {interval_ms}ms", 'info')
        messagebox.showinfo("Applied", f"Status poll interval set to {interval_ms}ms\n\n(takes effect on next update)")

    def _create_motor_status_frames(self, parent):
        """Create motor status display frames"""
        status_container = ttk.Frame(parent)
        status_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self._create_single_motor_frame(status_container, 0, "Motor 0 Status")
        self._create_single_motor_frame(status_container, 1, "Motor 1 Status")

    def _create_single_motor_frame(self, parent, motor_id, title):
        """Create a single motor status frame"""
        motor_frame = ttk.LabelFrame(parent, text=title, padding=10)
        motor_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        pos_label = ttk.Label(motor_frame, text="Position: 0", font=('Arial', 12))
        pos_label.pack(anchor=tk.W, pady=2)
        
        status_label = ttk.Label(motor_frame, text="Status: IDLE")
        status_label.pack(anchor=tk.W, pady=2)
        
        error_label = ttk.Label(motor_frame, text="Errors: None")
        error_label.pack(anchor=tk.W, pady=2)
        
        hex_label = ttk.Label(motor_frame, text="Register: 0x0000", font=('Courier', 10))
        hex_label.pack(anchor=tk.W, pady=2)
        
        bits_frame = ttk.Frame(motor_frame)
        bits_frame.pack(fill=tk.X, pady=10)
        
        # Store references
        if motor_id == 0:
            self.motor0_pos_label = pos_label
            self.motor0_status_label = status_label
            self.motor0_error_label = error_label
            self.motor0_hex_label = hex_label
            self.motor0_status_bits = self._create_status_bits(bits_frame)
        else:
            self.motor1_pos_label = pos_label
            self.motor1_status_label = status_label
            self.motor1_error_label = error_label
            self.motor1_hex_label = hex_label
            self.motor1_status_bits = self._create_status_bits(bits_frame)

    def _create_quick_positions_matrix(self, parent):
        """Create 4x6 matrix of quick position buttons"""
        matrix_frame = ttk.LabelFrame(parent, text="Quick Positions", padding=10)
        matrix_frame.pack(fill=tk.X, padx=10, pady=10)
        
        matrix_container = ttk.Frame(matrix_frame)
        matrix_container.pack()
        
        for r in range(BUTTON_MATRIX_ROWS):
            for c in range(BUTTON_MATRIX_COLS):
                pos_num = r * BUTTON_MATRIX_COLS + c + 1
                pos_name = f"P{pos_num}"
                
                cell_frame = ttk.Frame(matrix_container)
                cell_frame.grid(row=r, column=c, padx=3, pady=3, sticky='ew')
                
                btn = ttk.Button(cell_frame, text=pos_name, width=8,
                                command=lambda name=pos_name: self._safe_go_to_position(name))
                btn.pack()
                
                coord_label = ttk.Label(cell_frame, text="(N/A, N/A)", font=('Arial', 8))
                coord_label.pack()
                self.matrix_coord_labels[pos_name] = coord_label
        
        home_cell_frame = ttk.Frame(matrix_frame)
        home_cell_frame.pack(pady=5)
        ttk.Button(home_cell_frame, text="Home", width=10,
                  command=lambda: self._safe_go_to_position("Home")).pack()
        self.home_coord_label = ttk.Label(home_cell_frame, text="(N/A, N/A)", font=('Arial', 8))
        self.home_coord_label.pack()

    def _create_status_control_buttons(self, parent):
        """Create status control buttons"""
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(control_frame, text="Read Status", command=self.read_status_once).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Clear Errors", command=self.clear_errors).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Soft Stop", command=self.soft_stop, 
                style='Danger.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="HiZ (Disable)", command=self.hiz_disable, 
                style='Danger.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="EMERGENCY STOP", command=self.emergency_stop, 
                style='Danger.TButton').pack(side=tk.LEFT, padx=20)

    def soft_stop(self):
        """Soft stop all motors (coasts to stop)"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
        
        self.send_command("STOP")
        self.log_console("Soft stop activated - motors coasting...", 'info')

    def _create_status_control_buttons(self, parent):
        """Create status control buttons"""
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(control_frame, text="Read Status", command=self.read_status_once).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Clear Errors", command=self.clear_errors).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Soft Stop", command=self.soft_stop, 
                style='Danger.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="HiZ (Disable)", command=self.hiz_disable, 
                style='Danger.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="EMERGENCY STOP", command=self.emergency_stop, 
                style='Danger.TButton').pack(side=tk.LEFT, padx=20)

    def soft_stop(self):
        """Soft stop all motors (coasts to stop)"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
        
        self.send_command("STOP")
        self.log_console("Soft stop activated - motors coasting...", 'info')

    def hiz_disable(self):
        """Disable motors (high impedance state)"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
        
        self.send_command("HIZ")
        self.log_console("HiZ enabled - power bridges disabled", 'info')

    def _create_status_bits(self, parent):
        """Create status bit indicators"""
        bits = {}
        bit_names = ['HIZ', 'BUSY', 'SW_F', 'SW_EVN', 'DIR', 'MOT_STAT', 'CMD_ERR', 
                     'STALL_A', 'STALL_B', 'OCD', 'TH_WARN', 'UVLO', 'WRONG_CMD', 'NOTPERF_CMD']
        
        row_frame = None
        for i, name in enumerate(bit_names):
            if i % 7 == 0:
                row_frame = ttk.Frame(parent)
                row_frame.pack(fill=tk.X)
            
            bit_frame = ttk.Frame(row_frame)
            bit_frame.pack(side=tk.LEFT, padx=2)
            
            canvas = tk.Canvas(bit_frame, width=12, height=12)
            canvas.pack()
            indicator = canvas.create_oval(2, 2, 12, 12, fill='gray', outline='black')
            
            ttk.Label(bit_frame, text=name, font=('Arial', 8)).pack()
            bits[name] = (canvas, indicator)
        
        return bits

    def setup_control_tab(self, parent):
        """Setup manual control tab"""
        self._create_motor_status_frames(parent)
        self._create_jog_controls(parent)
        self._create_direct_position_controls(parent)

    def _create_jog_controls(self, parent):
        """Create jog control frame"""
        jog_frame = ttk.LabelFrame(parent, text="Manual Jog Control", padding=10)
        jog_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self._create_motor_jog_frame(jog_frame, 0)
        self._create_motor_jog_frame(jog_frame, 1)

    def _create_motor_jog_frame(self, parent, motor_id):
        """Create jog controls for a single motor"""
        m_frame = ttk.LabelFrame(parent, text=f"Motor {motor_id}", padding=10)
        m_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        step_frame = ttk.Frame(m_frame)
        step_frame.pack(fill=tk.X, pady=5)
        ttk.Label(step_frame, text="Step Size:").pack(side=tk.LEFT, padx=5)
        
        step_var = tk.IntVar(value=100)
        ttk.Spinbox(step_frame, from_=1, to=5000, textvariable=step_var, width=10).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(m_frame, text="<<< Fast (-10x)", 
                  command=lambda: self.jog_motor(motor_id, -step_var.get()*10)).pack(fill=tk.X, pady=2)
        ttk.Button(m_frame, text="<< Backward", 
                  command=lambda: self.jog_motor(motor_id, -step_var.get())).pack(fill=tk.X, pady=2)
        ttk.Button(m_frame, text=">> Forward", 
                  command=lambda: self.jog_motor(motor_id, step_var.get())).pack(fill=tk.X, pady=2)
        ttk.Button(m_frame, text=">>> Fast (+10x)", 
                  command=lambda: self.jog_motor(motor_id, step_var.get()*10)).pack(fill=tk.X, pady=2)
        
        if motor_id == 0:
            self.m0_step_var = step_var
        else:
            self.m1_step_var = step_var

    def _create_direct_position_controls(self, parent):
        """Create direct position control frame"""
        pos_frame = ttk.LabelFrame(parent, text="Direct Position Control", padding=10)
        pos_frame.pack(fill=tk.X, padx=10, pady=10)
        
        for motor_id in [0, 1]:
            m_pos_frame = ttk.Frame(pos_frame)
            m_pos_frame.pack(fill=tk.X, pady=5)
            ttk.Label(m_pos_frame, text=f"Motor {motor_id} Target:").pack(side=tk.LEFT, padx=5)
            
            target_var = tk.IntVar(value=0)
            ttk.Entry(m_pos_frame, textvariable=target_var, width=10).pack(side=tk.LEFT, padx=5)
            
            if motor_id == 0:
                self.m0_target_var = target_var
            else:
                self.m1_target_var = target_var
        
        button_frame = ttk.Frame(pos_frame)
        button_frame.pack(fill=tk.X, pady=10)
        ttk.Button(button_frame, text="Move to Position", 
                  command=self.move_to_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Home (Hardware)", 
                  command=self.home).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Reset Position Counters", 
                  command=self.reset_position).pack(side=tk.LEFT, padx=5)

    def setup_position_tab(self, parent):
        """Setup position management tab"""
        self._create_save_position_frame(parent)
        self._create_position_list_frame(parent)

    def _create_save_position_frame(self, parent):
        """Create frame for saving positions (both current and manual entry)"""
        save_frame = ttk.LabelFrame(parent, text="Save Position", padding=10)
        save_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Current position section
        current_frame = ttk.LabelFrame(save_frame, text="From Current Position", padding=5)
        current_frame.pack(fill=tk.X, pady=5)
        
        input_frame = ttk.Frame(current_frame)
        input_frame.pack(fill=tk.X)
        
        ttk.Label(input_frame, text="Position Name:").pack(side=tk.LEFT, padx=5)
        self.pos_name_var = tk.StringVar()
        ttk.Entry(input_frame, textvariable=self.pos_name_var, width=20).pack(side=tk.LEFT, padx=5)
        ttk.Button(input_frame, text="Save Current", command=self.save_current_position).pack(side=tk.LEFT, padx=5)
        
        self.current_pos_label = ttk.Label(current_frame, text="Current: M0=0, M1=0", font=('Arial', 11))
        self.current_pos_label.pack(anchor=tk.W, pady=5)
        
        # Manual entry section
        manual_frame = ttk.LabelFrame(save_frame, text="Manual Entry", padding=5)
        manual_frame.pack(fill=tk.X, pady=5)
        
        m0_frame = ttk.Frame(manual_frame)
        m0_frame.pack(fill=tk.X, pady=3)
        ttk.Label(m0_frame, text="Motor 0 Position:").pack(side=tk.LEFT, padx=5)
        self.manual_m0_var = tk.IntVar(value=0)
        ttk.Entry(m0_frame, textvariable=self.manual_m0_var, width=15).pack(side=tk.LEFT, padx=5)
        
        m1_frame = ttk.Frame(manual_frame)
        m1_frame.pack(fill=tk.X, pady=3)
        ttk.Label(m1_frame, text="Motor 1 Position:").pack(side=tk.LEFT, padx=5)
        self.manual_m1_var = tk.IntVar(value=0)
        ttk.Entry(m1_frame, textvariable=self.manual_m1_var, width=15).pack(side=tk.LEFT, padx=5)
        
        manual_name_frame = ttk.Frame(manual_frame)
        manual_name_frame.pack(fill=tk.X, pady=3)
        ttk.Label(manual_name_frame, text="Position Name:").pack(side=tk.LEFT, padx=5)
        self.manual_pos_name_var = tk.StringVar()
        ttk.Entry(manual_name_frame, textvariable=self.manual_pos_name_var, width=20).pack(side=tk.LEFT, padx=5)
        ttk.Button(manual_name_frame, text="Save Manual", command=self.save_manual_position).pack(side=tk.LEFT, padx=5)

    def save_manual_position(self):
        """Save manually entered position"""
        name = self.manual_pos_name_var.get().strip()
        if not name:
            messagebox.showerror("Error", "Please enter a position name")
            return
        
        # Validate name
        if not name.replace('_', '').replace('-', '').isalnum():
            messagebox.showerror("Error", "Position name can only contain letters, numbers, hyphens and underscores")
            return
        
        try:
            m0_pos = max(MIN_POSITION, min(MAX_POSITION, self.manual_m0_var.get()))
            m1_pos = max(MIN_POSITION, min(MAX_POSITION, self.manual_m1_var.get()))
            
            self.saved_positions[name] = {
                'motor0': m0_pos,
                'motor1': m1_pos
            }
            self.save_positions()
            self.refresh_position_list()
            self.manual_pos_name_var.set("")
            self.manual_m0_var.set(0)
            self.manual_m1_var.set(0)
            
            self.log_console(f"✓ Saved manual position '{name}' → M0:{m0_pos:,}, M1:{m1_pos:,}", 'info')
            messagebox.showinfo("Success", f"Position '{name}' saved successfully!")
        except Exception as e:
            logger.exception("Error saving manual position")
            messagebox.showerror("Error", f"Failed to save position:\n{str(e)}")

    def _create_position_list_frame(self, parent):
        """Create position list display frame"""
        list_frame = ttk.LabelFrame(parent, text="Saved Positions", padding=10)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
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
        
        self._create_sequence_list(main_pane)
        self._create_sequence_editor(main_pane)

    def _create_sequence_list(self, parent):
        """Create sequence list panel"""
        seq_list_frame = ttk.LabelFrame(parent, text="Sequences", padding=10)
        parent.add(seq_list_frame, weight=1)
        
        self.sequence_tree = ttk.Treeview(seq_list_frame, columns=('Steps',), show='tree headings')
        self.sequence_tree.heading('#0', text='Name')
        self.sequence_tree.heading('Steps', text='Steps')
        self.sequence_tree.pack(fill=tk.BOTH, expand=True)
        self.sequence_tree.bind('<<TreeviewSelect>>', self.on_sequence_select)
        
        seq_btn_frame = ttk.Frame(seq_list_frame)
        seq_btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(seq_btn_frame, text="▶ Run", command=self.run_sequence).pack(side=tk.LEFT, padx=2)
        ttk.Button(seq_btn_frame, text="⏹ Stop", command=self.stop_sequence).pack(side=tk.LEFT, padx=2)
        ttk.Button(seq_btn_frame, text="Delete", command=self.delete_sequence).pack(side=tk.LEFT, padx=2)
        
        self.refresh_sequence_list()

    def _create_sequence_editor(self, parent):
        """Create sequence editor panel"""
        editor_frame = ttk.LabelFrame(parent, text="Sequence Editor", padding=10)
        parent.add(editor_frame, weight=2)
        
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
        ttk.Spinbox(add_step_frame, from_=0, to=60, increment=0.1, 
                   textvariable=self.delay_var, width=6).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(add_step_frame, text="Add Step", command=self.add_step_to_sequence).pack(side=tk.LEFT)
        
        editor_btn_frame = ttk.Frame(editor_frame)
        editor_btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(editor_btn_frame, text="Remove Step", command=self.remove_step_from_sequence).pack(side=tk.LEFT)
        ttk.Button(editor_btn_frame, text="Clear All", command=self.clear_sequence_steps).pack(side=tk.LEFT, padx=5)
        ttk.Button(editor_btn_frame, text="Save Sequence", command=self.save_new_sequence).pack(side=tk.RIGHT)
        
        self.update_pos_to_add_combo()

    def setup_debug_tab(self, parent):
        """Setup debug console tab"""
        cmd_frame = ttk.Frame(parent)
        cmd_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(cmd_frame, text="Command:").pack(side=tk.LEFT, padx=5)
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var, width=50)
        cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        cmd_entry.bind('<Return>', lambda e: self.send_custom_command())
        ttk.Button(cmd_frame, text="Send", command=self.send_custom_command).pack(side=tk.LEFT, padx=5)
        ttk.Button(cmd_frame, text="Clear", command=self.clear_console).pack(side=tk.LEFT, padx=5)
        
        console_frame = ttk.LabelFrame(parent, text="Communication Log", padding=5)
        console_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        console_container = ttk.Frame(console_frame)
        console_container.pack(fill=tk.BOTH, expand=True)
        
        self.console_text = tk.Text(console_container, height=25, width=80, 
                                   bg='black', fg='green', font=('Courier', 9), wrap=tk.WORD)
        self.console_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(console_container, command=self.console_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.console_text.config(yscrollcommand=scrollbar.set)
        
        self.console_text.tag_config('tx', foreground='cyan')
        self.console_text.tag_config('rx', foreground='yellow')
        self.console_text.tag_config('error', foreground='red')
        self.console_text.tag_config('info', foreground='white')

    def refresh_ports(self):
        """Refresh available COM ports"""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                self.port_combo.current(0)
            self.log_console(f"Found {len(ports)} serial port(s)", 'info')
        except Exception as e:
            logger.exception("Error refreshing ports")
            self.log_console(f"Error refreshing ports: {str(e)}", 'error')

    def connect(self):
        """Connect to serial port"""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a COM port")
            return
        
        if self.running:
            messagebox.showwarning("Warning", "Already connected")
            return
        
        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
            self.running = True
            self.stop_sequence_flag.clear()
            self.conn_status.config(text="CONNECTED", style='Connected.TLabel')
            self.log_console(f"Connected to {port} at {BAUD_RATE} baud", 'info')
            
            # Start background threads
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
            self.cmd_thread = threading.Thread(target=self.process_commands, daemon=True)
            self.cmd_thread.start()
            
            self.parse_thread = threading.Thread(target=self.process_responses, daemon=True)
            self.parse_thread.start()
            
            # Start auto-refresh if enabled
            if self.auto_refresh_var.get():
                self.start_auto_refresh()
            
        except Exception as e:
            logger.exception("Connection error")
            messagebox.showerror("Connection Error", f"Failed to connect:\n{str(e)}")
            self.running = False

    def disconnect(self):
        """Disconnect from serial port"""
        if not self.running:
            return
            
        self.log_console("Disconnecting...", 'info')
        self.running = False
        self.stop_auto_refresh()
        
        # Wait briefly for threads to finish
        time.sleep(0.3)
        
        # Close serial port
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.log_console("Serial port closed", 'info')
            except Exception as e:
                logger.exception("Error closing serial port")
                self.log_console(f"Error closing port: {str(e)}", 'error')
        
        self.conn_status.config(text="DISCONNECTED", style='Disconnected.TLabel')
        self.log_console("Disconnected", 'info')

    def toggle_auto_refresh(self):
        """Toggle auto-refresh status updates"""
        if self.auto_refresh_var.get() and self.running:
            self.start_auto_refresh()
        else:
            self.stop_auto_refresh()

    def start_auto_refresh(self):
        """Start periodic status updates"""
        if self.running:
            self.auto_refresh_enabled = True
            self.schedule_status_update()
            self.log_console("Auto-refresh enabled", 'info')

    def stop_auto_refresh(self):
        """Stop periodic status updates"""
        self.auto_refresh_enabled = False
        if self.status_update_job:
            try:
                self.root.after_cancel(self.status_update_job)
            except:
                pass
            self.status_update_job = None
        self.log_console("Auto-refresh disabled", 'info')

    def schedule_status_update(self):
        """Schedule next status update"""
        if self.auto_refresh_enabled and self.running:
            self.read_status()
            # Schedule next update
            interval_ms = int(self.status_poll_interval * 1000)
            self.status_update_job = self.root.after(interval_ms, self.schedule_status_update)

    def read_status_once(self):
        """Manually request status update once"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
        self.read_status()

    def read_serial(self):
        """Read serial data in background thread"""
        buffer = ""
        while self.running:
            try:
                if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('ascii', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            # Queue for parsing in separate thread
                            self.response_queue.put(line)
                            # Log received data
                            self.root.after(0, lambda l=line: self.log_console(f"RX: {l}", 'rx'))
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    logger.exception("Read error")
                    self.root.after(0, lambda: self.log_console(f"Read error: {str(e)}", 'error'))
                time.sleep(0.1)

    def process_responses(self):
        """Process received responses in background thread"""
        while self.running:
            try:
                line = self.response_queue.get(timeout=0.1)
                self.parse_response(line)
            except Empty:
                continue
            except Exception as e:
                if self.running:
                    logger.exception("Response processing error")
                    self.root.after(0, lambda: self.log_console(f"Parse error: {str(e)}", 'error'))

    def process_commands(self):
        """Process command queue in background thread"""
        while self.running:
            try:
                command = self.command_queue.get(timeout=0.1)
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write((command + '\n').encode())
                    self.root.after(0, lambda c=command: self.log_console(f"TX: {c}", 'tx'))
            except Empty:
                continue
            except Exception as e:
                if self.running:
                    logger.exception("Send error")
                    self.root.after(0, lambda: self.log_console(f"Send error: {str(e)}", 'error'))

    def send_command(self, command):
        """Queue command for sending"""
        if not self.running:
            logger.warning(f"Attempted to send command while disconnected: {command}")
            return
        with self.command_lock:
            self.command_queue.put(command)

    def parse_response(self, data):
        """Parse response from STM32 with error handling"""
        try:
            parts = data.split(',')
            
            if parts[0] == "STATUS" and len(parts) >= 6:
                motor_id = int(parts[1])
                if motor_id not in [0, 1]:
                    raise ValueError(f"Invalid motor ID: {motor_id}")
                    
                position = int(parts[2])
                position = max(MIN_POSITION, min(MAX_POSITION, position))
                
                is_moving = bool(int(parts[3]))
                has_error = bool(int(parts[4]))
                status_reg = int(parts[5], 16)
                
                self._set_motor_state(motor_id, position=position, moving=is_moving, 
                                    error=has_error, status_reg=status_reg)
                self.root.after(0, lambda m=motor_id: self.update_motor_display(m))
                
            elif parts[0] == "ERROR":
                # ERROR format: ERROR,motor_id,error_type,description
                # Handle both valid and malformed ERROR messages gracefully
                try:
                    motor_id = int(parts[1]) if len(parts) > 1 else 0
                except (ValueError, IndexError):
                    # If motor_id is not a valid integer, log as system error
                    motor_id = 0
                    error_type = "PARSE_ERROR"
                    description = f"Malformed error: {','.join(parts[1:])}"
                    logger.warning(f"Motor {motor_id} Error: {error_type} - {description}")
                    error_text = f"Motor {motor_id} Error: {error_type} - {description}"
                    self.root.after(0, lambda txt=error_text: self.log_console(txt, 'error'))
                    return
                
                error_type = parts[2] if len(parts) > 2 else "UNKNOWN"
                description = parts[3] if len(parts) > 3 else ""
                logger.warning(f"Motor {motor_id} Error: {error_type} - {description}")
                error_text = f"Motor {motor_id} Error: {error_type} - {description}"
                self.root.after(0, lambda txt=error_text: self.log_console(txt, 'error'))
                
            elif parts[0] in ["OK", "DEBUG", "SYSTEM"]:
                # Info messages - already logged in RX
                pass
            else:
                logger.debug(f"Unhandled response: {data}")
                
        except Exception as e:
            # Capture error message before using in lambda to avoid scope issues
            error_message = f"Parse error: {str(e)}"
            logger.exception(f"Parse error on data '{data}'")
            self.root.after(0, lambda msg=error_message: self.log_console(msg, 'error'))

    def update_motor_display(self, motor_id):
        """Update motor status display - must be called from main thread"""
        state = self._get_motor_state(motor_id)
        position = state['position']
        is_moving = state['moving']
        has_error = state['error']
        status_reg = state['status_reg']
        
        if motor_id == 0:
            labels = (self.motor0_pos_label, self.motor0_status_label, 
                     self.motor0_error_label, self.motor0_hex_label)
            bits = self.motor0_status_bits
        else:
            labels = (self.motor1_pos_label, self.motor1_status_label, 
                     self.motor1_error_label, self.motor1_hex_label)
            bits = self.motor1_status_bits
        
        labels[0].config(text=f"Position: {position:,}")
        labels[1].config(text=f"Status: {'MOVING' if is_moving else 'IDLE'}")
        labels[2].config(text=f"Errors: {'⚠ YES' if has_error else '✓ None'}")
        labels[3].config(text=f"Register: 0x{status_reg:04X}")
        
        style = 'Moving.TLabel' if is_moving else 'TLabel'
        labels[1].config(style=style)
        
        if has_error:
            labels[2].config(style='Error.TLabel')
        else:
            labels[2].config(style='TLabel')
        
        self.update_status_bits(motor_id, status_reg, bits)
        self.current_pos_label.config(
            text=f"Current: M0={self._get_motor_state(0)['position']:,}, M1={self._get_motor_state(1)['position']:,}"
        )

    def update_status_bits(self, motor_id, status_reg, bits):
        """Update status bit indicators"""
        bit_map = {
            'HIZ': (status_reg >> 0) & 1,
            'BUSY': not ((status_reg >> 1) & 1),
            'SW_F': (status_reg >> 2) & 1,
            'SW_EVN': (status_reg >> 3) & 1,
            'DIR': (status_reg >> 4) & 1,
            'CMD_ERR': (status_reg >> 7) & 1,
            'STALL_A': not ((status_reg >> 12) & 1),
            'STALL_B': not ((status_reg >> 13) & 1),
            'OCD': not ((status_reg >> 15) & 1),
            'UVLO': not ((status_reg >> 9) & 1),
        }
        
        for name, (canvas, indicator) in bits.items():
            if name in bit_map:
                is_active = bit_map[name]
                if name == 'BUSY':
                    color = 'green' if is_active else 'gray'
                elif name in ['HIZ', 'CMD_ERR', 'STALL_A', 'STALL_B', 'OCD', 'UVLO']:
                    color = 'red' if is_active else 'gray'
                else:
                    color = 'yellow' if is_active else 'gray'
                canvas.itemconfig(indicator, fill=color)

    def log_console(self, message, tag='info'):
        """Log message to console - thread-safe"""
        # Check if console exists yet (it's created in debug tab setup)
        if not hasattr(self, 'console_text'):
            # Console not ready yet, just log to logger
            logger.info(message)
            return
            
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.console_text.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        self.console_text.see(tk.END)
        
        # Limit console size
        lines = int(self.console_text.index('end-1c').split('.')[0])
        if lines > 1000:
            self.console_text.delete('1.0', '100.0')

    def clear_console(self):
        """Clear console"""
        self.console_text.delete(1.0, tk.END)
        self.log_console("Console cleared", 'info')

    def send_custom_command(self):
        """Send custom command"""
        command = self.cmd_var.get().strip()
        if command:
            self.send_command(command)
            self.cmd_var.set("")

    def read_status(self):
        """Request status update with rate limiting"""
        now = time.time()
        if now - self.last_status_request > self.status_poll_interval:
            self.send_command("STATUS")
            self.last_status_request = now

    def clear_errors(self):
        """Clear error flags"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
        self.send_command("STATUS,CLEAR")
        self.log_console("Clearing error flags...", 'info')

    def emergency_stop(self):
        """Emergency stop all motors"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
        
        self.stop_sequence_flag.set()
        self.sequence_running = False
        self.send_command("STOP")
        self.log_console("⚠⚠⚠ EMERGENCY STOP ACTIVATED ⚠⚠⚠", 'error')
        messagebox.showwarning("Emergency Stop", "All motors stopped!")

    def jog_motor(self, motor_id, steps):
        """Jog a single motor"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
            
        state = self._get_motor_state(motor_id)
        current_pos = state['position']
        new_pos = current_pos + steps
        new_pos = max(MIN_POSITION, min(MAX_POSITION, new_pos))
        
        if motor_id == 0:
            other_pos = self._get_motor_state(1)['position']
            self.send_command(f"MOVE,{new_pos},{other_pos}")
        else:
            other_pos = self._get_motor_state(0)['position']
            self.send_command(f"MOVE,{other_pos},{new_pos}")

    def move_to_position(self):
        """Move to specified position"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
            
        try:
            m0_target = max(MIN_POSITION, min(MAX_POSITION, self.m0_target_var.get()))
            m1_target = max(MIN_POSITION, min(MAX_POSITION, self.m1_target_var.get()))
            self.send_command(f"MOVE,{m0_target},{m1_target}")
            self.log_console(f"Moving to position ({m0_target}, {m1_target})", 'info')
        except Exception as e:
            logger.exception("Invalid position")
            messagebox.showerror("Error", f"Invalid position value: {str(e)}")

    def home(self):
        """Home motors (hardware home)"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
            
        self.send_command("HOME")
        self.m0_target_var.set(0)
        self.m1_target_var.set(0)
        self.log_console("Executing hardware homing...", 'info')

    def reset_position(self):
        """Reset position counters"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
            
        if messagebox.askyesno("Confirm Reset", "Reset position counters to zero?\n\nThis will not move the motors."):
            self.send_command("RESET")
            time.sleep(0.1)
            self.read_status()
            self.log_console("Position counters reset", 'info')

    def _safe_go_to_position(self, pos_name):
        """Safely go to named position with validation"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
            
        if pos_name not in self.saved_positions:
            messagebox.showwarning("Not Found", f"Position '{pos_name}' not saved yet.\n\nSave it in the Position Management tab.")
            return
        self.go_to_named_position(pos_name)

    def go_to_named_position(self, pos_name):
        """Move to a named saved position"""
        if pos_name not in self.saved_positions:
            self.log_console(f"Position '{pos_name}' not found", 'error')
            return
            
        pos = self.saved_positions[pos_name]
        motor0_pos = pos.get('motor0', 0)
        motor1_pos = pos.get('motor1', 0)
        
        self.send_command(f"MOVE,{motor0_pos},{motor1_pos}")
        self.m0_target_var.set(motor0_pos)
        self.m1_target_var.set(motor1_pos)
        self.log_console(f"Moving to '{pos_name}' → M0:{motor0_pos:,}, M1:{motor1_pos:,}", 'info')

    def save_current_position(self):
        """Save current position"""
        name = self.pos_name_var.get().strip()
        if not name:
            messagebox.showerror("Error", "Please enter a position name")
            return
        
        # Validate name
        if not name.replace('_', '').replace('-', '').isalnum():
            messagebox.showerror("Error", "Position name can only contain letters, numbers, hyphens and underscores")
            return
        
        state0 = self._get_motor_state(0)
        state1 = self._get_motor_state(1)
        
        self.saved_positions[name] = {
            'motor0': state0['position'],
            'motor1': state1['position']
        }
        self.save_positions()
        self.refresh_position_list()
        self.pos_name_var.set("")
        self.log_console(f"✓ Saved position '{name}' → M0:{state0['position']:,}, M1:{state1['position']:,}", 'info')
        messagebox.showinfo("Success", f"Position '{name}' saved successfully!")

    def refresh_position_list(self):
        """Refresh position list display"""
        for item in self.position_tree.get_children():
            self.position_tree.delete(item)
        
        sorted_positions = sorted(self.saved_positions.items())
        for name, pos in sorted_positions:
            self.position_tree.insert('', 'end', text=name, 
                                     values=(f"{pos['motor0']:,}", f"{pos['motor1']:,}"))
        
        self.update_matrix_labels()
        self.update_pos_to_add_combo()

    def update_matrix_labels(self):
        """Update quick position button labels"""
        for pos_name, label in self.matrix_coord_labels.items():
            if pos_name in self.saved_positions:
                coords = self.saved_positions[pos_name]
                label.config(text=f"({coords['motor0']:,}, {coords['motor1']:,})", foreground='green')
            else:
                label.config(text="(Not Set)", foreground='gray')
        
        if self.home_coord_label:
            if "Home" in self.saved_positions:
                coords = self.saved_positions["Home"]
                self.home_coord_label.config(text=f"({coords['motor0']:,}, {coords['motor1']:,})", foreground='green')
            else:
                self.home_coord_label.config(text="(Not Set)", foreground='gray')

    def go_to_saved_position(self):
        """Go to selected saved position"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
            
        selection = self.position_tree.selection()
        if not selection:
            messagebox.showerror("Error", "Please select a position")
            return
        
        item = self.position_tree.item(selection[0])
        self.go_to_named_position(item['text'])

    def on_position_double_click(self, event):
        """Handle double-click on position"""
        if self.running:
            self.go_to_saved_position()

    def delete_position(self):
        """Delete selected position"""
        selection = self.position_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a position to delete")
            return
        
        item = self.position_tree.item(selection[0])
        name = item['text']
        
        if messagebox.askyesno("Confirm Delete", f"Delete position '{name}'?"):
            if name in self.saved_positions:
                del self.saved_positions[name]
                self.save_positions()
                self.refresh_position_list()
                self.log_console(f"Deleted position '{name}'", 'info')

    def export_positions(self):
        """Export positions to file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Export Positions"
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.saved_positions, f, indent=2)
                messagebox.showinfo("Success", f"Exported {len(self.saved_positions)} positions")
                self.log_console(f"Exported positions to {filename}", 'info')
            except Exception as e:
                logger.exception("Export error")
                messagebox.showerror("Error", f"Failed to export:\n{str(e)}")

    def import_positions(self):
        """Import positions from file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Import Positions"
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    imported = json.load(f)
                
                # Validate imported data
                if not isinstance(imported, dict):
                    raise ValueError("Invalid file format")
                
                count = len(imported)
                self.saved_positions.update(imported)
                self.save_positions()
                self.refresh_position_list()
                messagebox.showinfo("Success", f"Imported {count} positions")
                self.log_console(f"Imported {count} positions from {filename}", 'info')
            except Exception as e:
                logger.exception("Import error")
                messagebox.showerror("Error", f"Failed to import:\n{str(e)}")

    def load_positions(self):
        """Load saved positions from file"""
        if os.path.exists(POSITIONS_FILE):
            try:
                with open(POSITIONS_FILE, 'r') as f:
                    self.saved_positions = json.load(f)
                logger.info(f"Loaded {len(self.saved_positions)} positions")
            except Exception as e:
                logger.exception("Error loading positions")
                self.saved_positions = {}

    def save_positions(self):
        """Save positions to file"""
        try:
            with open(POSITIONS_FILE, 'w') as f:
                json.dump(self.saved_positions, f, indent=2)
        except Exception as e:
            logger.exception("Failed to save positions")
            self.log_console(f"Failed to save positions: {str(e)}", 'error')

    def load_sequences(self):
        """Load sequences from file"""
        if os.path.exists(SEQUENCES_FILE):
            try:
                with open(SEQUENCES_FILE, 'r') as f:
                    self.sequences = json.load(f)
                logger.info(f"Loaded {len(self.sequences)} sequences")
            except Exception as e:
                logger.exception("Error loading sequences")
                self.sequences = {}

    def save_sequences(self):
        """Save sequences to file"""
        try:
            with open(SEQUENCES_FILE, 'w') as f:
                json.dump(self.sequences, f, indent=2)
        except Exception as e:
            logger.exception("Failed to save sequences")
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
        
        # Clear and repopulate steps
        for item in self.sequence_steps_tree.get_children():
            self.sequence_steps_tree.delete(item)
        
        for step in self.sequences.get(seq_name, []):
            self.sequence_steps_tree.insert('', 'end', 
                                           values=(step['pos_name'], step['delay']))

    def update_pos_to_add_combo(self):
        """Update the combobox with saved positions"""
        pos_names = sorted(self.saved_positions.keys())
        self.pos_to_add_combo['values'] = pos_names
        if pos_names and not self.pos_to_add_combo.get():
            self.pos_to_add_combo.current(0)

    def add_step_to_sequence(self):
        """Add a step to the editor tree"""
        pos_name = self.pos_to_add_combo.get()
        delay = self.delay_var.get()
        if not pos_name:
            messagebox.showwarning("Warning", "Please select a position")
            return
        self.sequence_steps_tree.insert('', 'end', values=(pos_name, delay))

    def remove_step_from_sequence(self):
        """Remove selected step from editor"""
        selection = self.sequence_steps_tree.selection()
        if selection:
            self.sequence_steps_tree.delete(selection[0])
        else:
            messagebox.showwarning("Warning", "Please select a step to remove")

    def clear_sequence_steps(self):
        """Clear all steps from editor"""
        if messagebox.askyesno("Confirm Clear", "Remove all steps from the editor?"):
            for item in self.sequence_steps_tree.get_children():
                self.sequence_steps_tree.delete(item)

    def save_new_sequence(self):
        """Save or update a sequence"""
        name = self.seq_name_var.get().strip()
        if not name:
            messagebox.showerror("Error", "Please enter a sequence name")
            return
        
        steps = []
        for item in self.sequence_steps_tree.get_children():
            values = self.sequence_steps_tree.item(item, 'values')
            steps.append({'pos_name': values[0], 'delay': float(values[1])})
        
        if not steps:
            messagebox.showwarning("Warning", "Sequence has no steps")
            return
        
        self.sequences[name] = steps
        self.save_sequences()
        self.refresh_sequence_list()
        self.log_console(f"✓ Sequence '{name}' saved with {len(steps)} steps", 'info')
        messagebox.showinfo("Success", f"Sequence '{name}' saved successfully!")

    def delete_sequence(self):
        """Delete selected sequence"""
        selection = self.sequence_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a sequence to delete")
            return
        
        name = self.sequence_tree.item(selection[0])['text']
        if messagebox.askyesno("Confirm Delete", f"Delete sequence '{name}'?"):
            if name in self.sequences:
                del self.sequences[name]
                self.save_sequences()
                self.refresh_sequence_list()
                self.log_console(f"Deleted sequence '{name}'", 'info')

    def stop_sequence(self):
        """Stop currently running sequence"""
        if self.sequence_running:
            self.stop_sequence_flag.set()
            self.log_console("Stopping sequence...", 'error')
        else:
            messagebox.showinfo("Info", "No sequence is currently running")

    def run_sequence(self):
        """Run the selected sequence"""
        if not self.running:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            return
            
        if self.sequence_running:
            messagebox.showwarning("Busy", "A sequence is already running.\nUse the Stop button to cancel it.")
            return
        
        selection = self.sequence_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a sequence to run")
            return
        
        name = self.sequence_tree.item(selection[0])['text']
        steps = self.sequences.get(name, [])
        
        if not steps:
            messagebox.showwarning("Warning", "Sequence has no steps")
            return
        
        # Validate all positions exist
        missing = [s['pos_name'] for s in steps if s['pos_name'] not in self.saved_positions]
        if missing:
            messagebox.showerror("Error", 
                f"Sequence contains undefined positions:\n{', '.join(missing)}\n\nPlease update the sequence.")
            return
        
        self.sequence_running = True
        self.stop_sequence_flag.clear()
        thread = threading.Thread(target=self._sequence_worker, args=(name, steps), daemon=True)
        thread.start()

    def _sequence_worker(self, name, steps):
        """Background worker to execute sequence"""
        self.root.after(0, lambda: self.log_console(f"▶ Starting sequence '{name}' ({len(steps)} steps)...", 'info'))
        
        try:
            for i, step in enumerate(steps):
                if self.stop_sequence_flag.is_set():
                    self.root.after(0, lambda: self.log_console("⏹ Sequence stopped by user", 'error'))
                    break
                
                pos_name = step['pos_name']
                delay = step['delay']
                
                if pos_name not in self.saved_positions:
                    self.root.after(0, lambda p=pos_name, n=i: 
                        self.log_console(f"Step {n+1}: Position '{p}' not found - skipping", 'error'))
                    continue
                
                # Move to position
                self.root.after(0, lambda p=pos_name, n=i, t=len(steps): 
                    self.log_console(f"Step {n+1}/{t}: Moving to '{p}'...", 'info'))
                self.go_to_named_position(pos_name)
                
                # Wait for motors to stop moving
                time.sleep(0.2)  # Initial settling time
                timeout = 30  # 30 second timeout
                start_time = time.time()
                
                while (self._get_motor_state(0)['moving'] or self._get_motor_state(1)['moving']):
                    if self.stop_sequence_flag.is_set() or (time.time() - start_time > timeout):
                        break
                    self.read_status()
                    time.sleep(SEQUENCE_POLL_INTERVAL)
                
                if self.stop_sequence_flag.is_set():
                    self.root.after(0, lambda: self.log_console("⏹ Sequence stopped by user", 'error'))
                    break
                
                # Delay before next step
                if delay > 0 and i < len(steps) - 1:  # Don't delay after last step
                    self.root.after(0, lambda d=delay: 
                        self.log_console(f"Waiting {d} seconds...", 'info'))
                    
                    # Break delay into smaller chunks to allow cancellation
                    delay_chunks = int(delay / 0.1)
                    for _ in range(delay_chunks):
                        if self.stop_sequence_flag.is_set():
                            break
                        time.sleep(0.1)
            
            if not self.stop_sequence_flag.is_set():
                self.root.after(0, lambda: self.log_console(f"✓ Sequence '{name}' completed successfully!", 'info'))
                self.root.after(0, lambda: messagebox.showinfo("Complete", f"Sequence '{name}' finished!"))
        
        except Exception as e:
            logger.exception("Sequence execution error")
            self.root.after(0, lambda: self.log_console(f"Sequence error: {str(e)}", 'error'))
        
        finally:
            self.sequence_running = False
            self.stop_sequence_flag.clear()

    def on_closing(self):
        """Handle window close event"""
        if self.sequence_running:
            if not messagebox.askyesno("Sequence Running", 
                "A sequence is currently running.\n\nAre you sure you want to exit?"):
                return
        
        if self.running:
            if messagebox.askyesno("Disconnect", "Disconnect and exit?"):
                self.disconnect()
                time.sleep(0.2)
                self.root.destroy()
        else:
            self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()