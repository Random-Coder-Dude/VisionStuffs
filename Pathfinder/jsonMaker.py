import tkinter as tk
from tkinter import filedialog, messagebox
import json
from PIL import Image, ImageTk
from collections import deque
import copy


class FieldGridEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("Field Grid Editor")
        self.root.configure(bg="#1e1e2e")

        # Field settings
        self.field_width = 16.54
        self.field_height = 8.07
        self.grid_cols = 100
        self.grid_rows = 52
        self.grid = [[1] * self.grid_cols for _ in range(self.grid_rows)]

        # Canvas settings
        self.canvas_width = 800
        self.canvas_height = 400
        self.bg_image = None
        self.bg_photo = None

        # Interaction state
        self.is_drawing = False
        self.draw_value = 0
        self.brush_size = 1
        self.last_cell = None
        self.pending_changes = []
        
        # Undo/Redo
        self.history = deque(maxlen=50)
        self.redo_stack = deque(maxlen=50)
        self.save_state()

        self.build_ui()
        self.setup_bindings()
        self.draw_grid()

    # ================= UI =================

    def build_ui(self):
        # Top controls
        top = tk.Frame(self.root, bg="#1e1e2e", pady=6)
        top.pack(fill=tk.X, padx=10)

        # Field dimensions
        tk.Label(top, text="Width (m):", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.width_var = tk.StringVar(value=str(self.field_width))
        tk.Entry(top, textvariable=self.width_var, width=6,
                 bg="#313244", fg="#cdd6f4", insertbackground="white",
                 relief=tk.FLAT, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(2, 8))

        tk.Label(top, text="Height (m):", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.height_var = tk.StringVar(value=str(self.field_height))
        tk.Entry(top, textvariable=self.height_var, width=6,
                 bg="#313244", fg="#cdd6f4", insertbackground="white",
                 relief=tk.FLAT, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(2, 8))

        tk.Label(top, text="Cols:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.cols_var = tk.StringVar(value=str(self.grid_cols))
        tk.Entry(top, textvariable=self.cols_var, width=4,
                 bg="#313244", fg="#cdd6f4", insertbackground="white",
                 relief=tk.FLAT, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(2, 8))

        tk.Label(top, text="Rows:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.rows_var = tk.StringVar(value=str(self.grid_rows))
        tk.Entry(top, textvariable=self.rows_var, width=4,
                 bg="#313244", fg="#cdd6f4", insertbackground="white",
                 relief=tk.FLAT, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(2, 8))

        tk.Button(top, text="Apply", command=self.apply_settings,
                  bg="#89b4fa", fg="#1e1e2e", relief=tk.FLAT,
                  padx=10, font=("Segoe UI", 9, "bold"),
                  cursor="hand2").pack(side=tk.LEFT, padx=(0, 8))

        # Brush size
        tk.Label(top, text="Brush:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.brush_var = tk.StringVar(value="1")
        brush_spin = tk.Spinbox(top, from_=1, to=10, textvariable=self.brush_var,
                                width=3, bg="#313244", fg="#cdd6f4",
                                relief=tk.FLAT, font=("Segoe UI", 9),
                                command=self.update_brush)
        brush_spin.pack(side=tk.LEFT, padx=(2, 8))

        # Action buttons
        for text, cmd, color in [
            ("Load Image", self.load_image, "#a6e3a1"),
            ("Clear", self.clear_all, "#f38ba8"),
            ("Fill", self.fill_all, "#fab387"),
            ("Load", self.load_json, "#cba6f7"),
            ("Save", self.save_json, "#89dceb"),
        ]:
            tk.Button(top, text=text, command=cmd,
                      bg=color, fg="#1e1e2e", relief=tk.FLAT,
                      padx=8, font=("Segoe UI", 9, "bold"),
                      cursor="hand2").pack(side=tk.LEFT, padx=2)

        # Canvas
        canvas_frame = tk.Frame(self.root, bg="#1e1e2e")
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 5))

        self.canvas = tk.Canvas(canvas_frame,
                                width=self.canvas_width,
                                height=self.canvas_height,
                                bg="#181825",
                                highlightthickness=0,
                                cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Status bar
        self.status_var = tk.StringVar(value="Left=Block | Right=Clear | Scroll=Brush | Ctrl+Z=Undo | Ctrl+Y=Redo")
        tk.Label(self.root, textvariable=self.status_var,
                 bg="#181825", fg="#6c7086", anchor=tk.W,
                 font=("Segoe UI", 9)).pack(fill=tk.X, padx=10, pady=(0, 6))

    def setup_bindings(self):
        # Mouse
        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<ButtonPress-3>", self.on_right_press)
        self.canvas.bind("<B3-Motion>", self.on_right_drag)
        self.canvas.bind("<ButtonRelease-3>", self.on_release)
        self.canvas.bind("<Configure>", self.on_resize)
        self.canvas.bind("<MouseWheel>", self.on_scroll)
        
        # Keyboard
        self.root.bind("<Control-z>", lambda e: self.undo())
        self.root.bind("<Control-y>", lambda e: self.redo())
        self.root.bind("<Control-s>", lambda e: self.save_json())
        self.root.bind("<Control-o>", lambda e: self.load_json())

    # ================= Undo/Redo =================

    def save_state(self):
        self.history.append(copy.deepcopy(self.grid))
        self.redo_stack.clear()

    def undo(self):
        if len(self.history) > 1:
            self.redo_stack.append(self.history.pop())
            self.grid = copy.deepcopy(self.history[-1])
            self.draw_grid()
            self.update_status()

    def redo(self):
        if self.redo_stack:
            state = self.redo_stack.pop()
            self.history.append(state)
            self.grid = copy.deepcopy(state)
            self.draw_grid()
            self.update_status()

    # ================= Grid Logic =================

    def apply_settings(self):
        try:
            self.field_width = float(self.width_var.get())
            self.field_height = float(self.height_var.get())
            new_cols = int(self.cols_var.get())
            new_rows = int(self.rows_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid values!")
            return

        new_grid = [[1] * new_cols for _ in range(new_rows)]
        for r in range(min(new_rows, self.grid_rows)):
            for c in range(min(new_cols, self.grid_cols)):
                new_grid[r][c] = self.grid[r][c]

        self.grid_cols = new_cols
        self.grid_rows = new_rows
        self.grid = new_grid
        self.save_state()
        self.draw_grid()
        self.update_status()

    def update_brush(self):
        try:
            self.brush_size = int(self.brush_var.get())
        except ValueError:
            self.brush_size = 1

    def cell_size(self):
        w = self.canvas.winfo_width() or self.canvas_width
        h = self.canvas.winfo_height() or self.canvas_height
        return w / self.grid_cols, h / self.grid_rows

    def canvas_to_cell(self, x, y):
        cw, ch = self.cell_size()
        col = int(x / cw)
        row = int(y / ch)
        col = max(0, min(self.grid_cols - 1, col))
        row = max(0, min(self.grid_rows - 1, row))
        return row, col

    def draw_grid(self):
        self.canvas.delete("grid")
        cw, ch = self.cell_size()

        # Background image
        if self.bg_image:
            w = self.canvas.winfo_width() or self.canvas_width
            h = self.canvas.winfo_height() or self.canvas_height
            resized = self.bg_image.resize((w, h), Image.LANCZOS)
            self.bg_photo = ImageTk.PhotoImage(resized)
            self.canvas.create_image(0, 0, anchor=tk.NW,
                                     image=self.bg_photo, tags="grid")

        # Draw only blocked cells for performance
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                if self.grid[r][c] == 0:
                    x1 = c * cw
                    y1 = r * ch
                    x2 = x1 + cw
                    y2 = y1 + ch
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2,
                        fill="#f38ba8",
                        outline="",
                        tags="grid"
                    )

        # Grid lines (lighter for readability)
        for c in range(0, self.grid_cols + 1, max(1, self.grid_cols // 50)):
            x = c * cw
            self.canvas.create_line(x, 0, x, self.grid_rows * ch,
                                   fill="#313244", tags="grid")
        for r in range(0, self.grid_rows + 1, max(1, self.grid_rows // 50)):
            y = r * ch
            self.canvas.create_line(0, y, self.grid_cols * cw, y,
                                   fill="#313244", tags="grid")

    def paint_cells(self, center_row, center_col, value):
        """Paint cells in a brush radius around center"""
        radius = self.brush_size // 2
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                r = center_row + dr
                c = center_col + dc
                if 0 <= r < self.grid_rows and 0 <= c < self.grid_cols:
                    if self.grid[r][c] != value:
                        self.grid[r][c] = value
                        self.pending_changes.append((r, c))

    def redraw_changed_cells(self):
        """Efficiently redraw only changed cells"""
        cw, ch = self.cell_size()
        for r, c in self.pending_changes:
            x1 = c * cw
            y1 = r * ch
            x2 = x1 + cw
            y2 = y1 + ch
            
            # Delete old cell
            self.canvas.delete(f"cell_{r}_{c}")
            
            # Draw new cell if blocked
            if self.grid[r][c] == 0:
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill="#f38ba8",
                    outline="",
                    tags=("grid", f"cell_{r}_{c}")
                )
        self.pending_changes.clear()

    # ================= Mouse =================

    def on_press(self, event):
        row, col = self.canvas_to_cell(event.x, event.y)
        self.draw_value = 0
        self.is_drawing = True
        self.last_cell = None
        self.paint_cells(row, col, 0)
        self.redraw_changed_cells()
        self.update_status(row, col)

    def on_drag(self, event):
        if self.is_drawing:
            row, col = self.canvas_to_cell(event.x, event.y)
            if (row, col) != self.last_cell:
                self.paint_cells(row, col, self.draw_value)
                self.redraw_changed_cells()
                self.last_cell = (row, col)
                self.update_status(row, col)

    def on_release(self, event):
        if self.is_drawing:
            self.save_state()
            self.is_drawing = False
            self.last_cell = None

    def on_right_press(self, event):
        row, col = self.canvas_to_cell(event.x, event.y)
        self.draw_value = 1
        self.is_drawing = True
        self.last_cell = None
        self.paint_cells(row, col, 1)
        self.redraw_changed_cells()
        self.update_status(row, col)

    def on_right_drag(self, event):
        self.on_drag(event)

    def on_scroll(self, event):
        delta = 1 if event.delta > 0 else -1
        self.brush_size = max(1, min(10, self.brush_size + delta))
        self.brush_var.set(str(self.brush_size))
        self.update_status()

    def on_resize(self, event):
        self.draw_grid()

    # ================= File IO =================

    def save_json(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")]
        )
        if not path:
            return

        flipped = list(reversed(self.grid))
        string_rows = ["".join(str(cell) for cell in row) for row in flipped]

        data = {
            "fieldWidth": self.field_width,
            "fieldHeight": self.field_height,
            "cols": self.grid_cols,
            "rows": self.grid_rows,
            "grid": string_rows
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        messagebox.showinfo("Saved", f"Grid saved to {path}")

    def load_json(self):
        path = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json")]
        )
        if not path:
            return

        with open(path, "r") as f:
            data = json.load(f)

        self.field_width = data["fieldWidth"]
        self.field_height = data["fieldHeight"]
        self.grid_cols = data["cols"]
        self.grid_rows = data["rows"]

        string_rows = data["grid"]
        numeric = [[int(ch) for ch in row] for row in string_rows]
        self.grid = list(reversed(numeric))

        self.width_var.set(str(self.field_width))
        self.height_var.set(str(self.field_height))
        self.cols_var.set(str(self.grid_cols))
        self.rows_var.set(str(self.grid_rows))

        self.save_state()
        self.draw_grid()
        self.update_status()

    # ================= Utilities =================

    def load_image(self):
        path = filedialog.askopenfilename(
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp")]
        )
        if path:
            self.bg_image = Image.open(path)
            self.draw_grid()

    def clear_all(self):
        self.grid = [[0] * self.grid_cols for _ in range(self.grid_rows)]
        self.save_state()
        self.draw_grid()
        self.update_status()

    def fill_all(self):
        self.grid = [[1] * self.grid_cols for _ in range(self.grid_rows)]
        self.save_state()
        self.draw_grid()
        self.update_status()

    def update_status(self, row=None, col=None):
        cw = self.field_width / self.grid_cols
        ch = self.field_height / self.grid_rows
        blocked = sum(row.count(0) for row in self.grid)
        total = self.grid_rows * self.grid_cols
        
        msg = f"Cell: {cw:.3f}m×{ch:.3f}m | Blocked: {blocked}/{total} | Brush: {self.brush_size}"
        
        if row is not None and col is not None:
            x = col * cw
            y = row * ch
            msg += f" | ({row},{col}) = ({x:.2f}m, {y:.2f}m)"
        
        self.status_var.set(msg)


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1000x650")
    app = FieldGridEditor(root)
    root.mainloop()