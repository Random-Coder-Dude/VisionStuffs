import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import math
from PIL import Image, ImageTk

class FieldGridEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("Field Grid Editor")
        self.root.configure(bg="#1e1e2e")

        # Field settings
        self.field_width = 16.54
        self.field_height = 8.21
        self.grid_cols = 33
        self.grid_rows = 16
        self.grid = [[1] * self.grid_cols for _ in range(self.grid_rows)]

        # Canvas settings
        self.canvas_width = 800
        self.canvas_height = 400
        self.bg_image = None
        self.bg_photo = None

        # Interaction state
        self.is_drawing = False
        self.draw_value = 0  # 0 = blocking, 1 = clearing

        self.build_ui()
        self.draw_grid()

    def build_ui(self):
        # Top bar
        top = tk.Frame(self.root, bg="#1e1e2e", pady=6)
        top.pack(fill=tk.X, padx=10)

        # Field dimensions
        tk.Label(top, text="Field Width (m):", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.width_var = tk.StringVar(value=str(self.field_width))
        tk.Entry(top, textvariable=self.width_var, width=6, bg="#313244", fg="#cdd6f4",
                 insertbackground="white", relief=tk.FLAT).pack(side=tk.LEFT, padx=(2, 10))

        tk.Label(top, text="Field Height (m):", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.height_var = tk.StringVar(value=str(self.field_height))
        tk.Entry(top, textvariable=self.height_var, width=6, bg="#313244", fg="#cdd6f4",
                 insertbackground="white", relief=tk.FLAT).pack(side=tk.LEFT, padx=(2, 10))

        tk.Label(top, text="Cols:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.cols_var = tk.StringVar(value=str(self.grid_cols))
        tk.Entry(top, textvariable=self.cols_var, width=4, bg="#313244", fg="#cdd6f4",
                 insertbackground="white", relief=tk.FLAT).pack(side=tk.LEFT, padx=(2, 10))

        tk.Label(top, text="Rows:", bg="#1e1e2e", fg="#cdd6f4", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.rows_var = tk.StringVar(value=str(self.grid_rows))
        tk.Entry(top, textvariable=self.rows_var, width=4, bg="#313244", fg="#cdd6f4",
                 insertbackground="white", relief=tk.FLAT).pack(side=tk.LEFT, padx=(2, 10))

        tk.Button(top, text="Apply", command=self.apply_settings,
                  bg="#89b4fa", fg="#1e1e2e", relief=tk.FLAT, padx=8,
                  font=("Segoe UI", 10, "bold"), cursor="hand2").pack(side=tk.LEFT, padx=(0, 10))

        # Buttons
        for text, cmd, color in [
            ("Load Image", self.load_image, "#a6e3a1"),
            ("Clear All", self.clear_all, "#f38ba8"),
            ("Fill All", self.fill_all, "#fab387"),
            ("Load JSON", self.load_json, "#cba6f7"),
            ("Save JSON", self.save_json, "#89dceb"),
        ]:
            tk.Button(top, text=text, command=cmd, bg=color, fg="#1e1e2e",
                      relief=tk.FLAT, padx=8, font=("Segoe UI", 10, "bold"),
                      cursor="hand2").pack(side=tk.LEFT, padx=3)

        # Canvas frame
        canvas_frame = tk.Frame(self.root, bg="#1e1e2e")
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        self.canvas = tk.Canvas(canvas_frame, width=self.canvas_width, height=self.canvas_height,
                                bg="#181825", highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<ButtonPress-3>", self.on_right_press)
        self.canvas.bind("<B3-Motion>", self.on_right_drag)
        self.canvas.bind("<Configure>", self.on_resize)

        # Status bar
        self.status_var = tk.StringVar(value="Left click = block | Right click = clear | Drag to paint")
        tk.Label(self.root, textvariable=self.status_var, bg="#181825", fg="#6c7086",
                 font=("Segoe UI", 9), anchor=tk.W).pack(fill=tk.X, padx=10, pady=(0, 6))

    def apply_settings(self):
        try:
            new_width = float(self.width_var.get())
            new_height = float(self.height_var.get())
            new_cols = int(self.cols_var.get())
            new_rows = int(self.rows_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid values!")
            return

        # Resize grid preserving existing data
        new_grid = [[1] * new_cols for _ in range(new_rows)]
        for r in range(min(new_rows, self.grid_rows)):
            for c in range(min(new_cols, self.grid_cols)):
                new_grid[r][c] = self.grid[r][c]

        self.field_width = new_width
        self.field_height = new_height
        self.grid_cols = new_cols
        self.grid_rows = new_rows
        self.grid = new_grid

        self.draw_grid()
        self.update_status()

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

        # Draw background image
        if self.bg_image:
            w = self.canvas.winfo_width() or self.canvas_width
            h = self.canvas.winfo_height() or self.canvas_height
            resized = self.bg_image.resize((w, h), Image.LANCZOS)
            self.bg_photo = ImageTk.PhotoImage(resized)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.bg_photo, tags="grid")

        # Draw cells
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                x1 = c * cw
                y1 = r * ch
                x2 = x1 + cw
                y2 = y1 + ch

                if self.grid[r][c] == 0:
                    fill = "#f38ba8"  # Red for blocked
                else:
                    fill = "" if self.bg_image else "#313244"

                outline = "#45475a"
                self.canvas.create_rectangle(x1, y1, x2, y2,
                                             fill=fill, outline=outline,
                                             stipple="gray25" if self.grid[r][c] == 0 and self.bg_image else "",
                                             tags="grid")

    def set_cell(self, row, col, value):
        if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
            self.grid[row][col] = value
            cw, ch = self.cell_size()
            x1 = col * cw
            y1 = row * ch
            x2 = x1 + cw
            y2 = y1 + ch

            # Redraw just this cell
            self.canvas.delete(f"cell_{row}_{col}")
            if value == 0:
                fill = "#f38ba8"
                stipple = ""
            else:
                fill = "" if self.bg_image else "#313244"
                stipple = ""

            self.canvas.create_rectangle(x1, y1, x2, y2,
                                         fill=fill, outline="#45475a",
                                         stipple=stipple,
                                         tags=("grid", f"cell_{row}_{col}"))
            self.update_status(row, col)

    def on_press(self, event):
        row, col = self.canvas_to_cell(event.x, event.y)
        self.draw_value = 0  # block
        self.is_drawing = True
        self.set_cell(row, col, self.draw_value)

    def on_drag(self, event):
        if self.is_drawing:
            row, col = self.canvas_to_cell(event.x, event.y)
            self.set_cell(row, col, self.draw_value)

    def on_release(self, event):
        self.is_drawing = False

    def on_right_press(self, event):
        row, col = self.canvas_to_cell(event.x, event.y)
        self.draw_value = 1  # clear
        self.is_drawing = True
        self.set_cell(row, col, self.draw_value)

    def on_right_drag(self, event):
        if self.is_drawing:
            row, col = self.canvas_to_cell(event.x, event.y)
            self.set_cell(row, col, self.draw_value)

    def on_resize(self, event):
        self.draw_grid()

    def load_image(self):
        path = filedialog.askopenfilename(filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp")])
        if path:
            self.bg_image = Image.open(path)
            self.draw_grid()

    def clear_all(self):
        self.grid = [[0] * self.grid_cols for _ in range(self.grid_rows)]
        self.draw_grid()

    def fill_all(self):
        self.grid = [[1] * self.grid_cols for _ in range(self.grid_rows)]
        self.draw_grid()

    def save_json(self):
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON files", "*.json")])
        if path:
            data = {
                "fieldWidth": self.field_width,
                "fieldHeight": self.field_height,
                "grid": self.grid
            }
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Saved", f"Grid saved to {path}")

    def load_json(self):
        path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if path:
            with open(path, "r") as f:
                data = json.load(f)
            self.field_width = data["fieldWidth"]
            self.field_height = data["fieldHeight"]
            self.grid = data["grid"]
            self.grid_rows = len(self.grid)
            self.grid_cols = len(self.grid[0]) if self.grid else 0

            self.width_var.set(str(self.field_width))
            self.height_var.set(str(self.field_height))
            self.rows_var.set(str(self.grid_rows))
            self.cols_var.set(str(self.grid_cols))

            self.draw_grid()

    def update_status(self, row=None, col=None):
        cw = self.field_width / self.grid_cols
        ch = self.field_height / self.grid_rows
        blocked = sum(1 for r in self.grid for c in r if c == 0)
        total = self.grid_rows * self.grid_cols
        base = f"Cell size: {cw:.2f}m x {ch:.2f}m | Blocked: {blocked}/{total}"
        if row is not None:
            x = col * cw
            y = row * ch
            base += f" | Cell ({row},{col}) = {x:.2f}m, {y:.2f}m"
        self.status_var.set(base)


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("900x550")
    app = FieldGridEditor(root)
    root.mainloop()