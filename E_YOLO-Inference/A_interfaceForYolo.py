import customtkinter as ctk
import os
import subprocess
import threading
import re
from PIL import Image

# --- Configuration ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")
ctk.set_widget_scaling(1.0)
ctk.set_window_scaling(1.0)

class YoloControlPanel(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Window Setup
        self.title("YOLO Control Center")
        self.geometry("1100x650")

        # --- LOAD ICONS ---
        icon_size = (40, 40)
        try:
            self.img_cam = ctk.CTkImage(light_image=Image.open("icon_camera.png"),
                                        dark_image=Image.open("icon_camera.png"), size=icon_size)
            self.img_robo = ctk.CTkImage(light_image=Image.open("icon_roboflow.png"),
                                         dark_image=Image.open("icon_roboflow.png"), size=icon_size)
            self.img_hammer = ctk.CTkImage(light_image=Image.open("icon_hammer.png"),
                                           dark_image=Image.open("icon_hammer.png"), size=icon_size)
            self.img_brain = ctk.CTkImage(light_image=Image.open("icon_brain.png"),
                                          dark_image=Image.open("icon_brain.png"), size=icon_size)
        except Exception:
            self.img_cam = None
            self.img_robo = None
            self.img_hammer = None
            self.img_brain = None

        # Grid Layout
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # --- 1. TOP STATUS BAR ---
        self.status_frame = ctk.CTkFrame(self, height=50, corner_radius=0)
        self.status_frame.grid(row=0, column=0, columnspan=3, sticky="ew")
        
        self.cam_label = ctk.CTkLabel(self.status_frame, text="Checking Cameras...", font=("Arial", 14, "bold"))
        self.cam_label.pack(pady=10)
        self.check_camera_status()

        # --- 2. LEFT SIDEBAR ---
        self.nav_frame = ctk.CTkFrame(self, width=150, corner_radius=0) 
        self.nav_frame.grid(row=1, column=0, sticky="nsew")
        self.nav_frame.grid_rowconfigure(6, weight=1)
        
        ctk.CTkLabel(self.nav_frame, text="").pack(pady=10) 

        self.create_nav_btn("Cam Cap", self.show_capture, self.img_cam)
        self.create_nav_btn("Pull Imgs", self.show_pull, self.img_robo)
        self.create_nav_btn("Run Train", self.show_train, self.img_hammer)
        self.create_nav_btn("Optimize", self.show_opt, self.img_brain)

        # --- 3. RIGHT SIDEBAR (Session Clipboard) ---
        self.clip_frame = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.clip_frame.grid(row=1, column=2, sticky="nsew")
        
        ctk.CTkLabel(self.clip_frame, text="Session Names", font=("Arial", 16, "bold")).pack(pady=10)
        self.clipboard_box = ctk.CTkTextbox(self.clip_frame, height=400)
        self.clipboard_box.pack(padx=10, pady=10, fill="both", expand=True)

        # --- 4. CENTER MAIN AREA ---
        self.main_frame = ctk.CTkFrame(self, corner_radius=10, fg_color="transparent")
        self.main_frame.grid(row=1, column=1, sticky="nsew", padx=20, pady=20)

        self.frames = {}
        self.create_frames()
        self.show_frame("Start")

    def create_nav_btn(self, text, command, icon_image):
        btn = ctk.CTkButton(self.nav_frame, text=text, image=icon_image, command=command, 
                            compound="top", height=80, corner_radius=8, fg_color="transparent", 
                            text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"), font=("Arial", 12))
        btn.pack(fill="x", pady=5, padx=5)
        return btn

    def create_frames(self):
        # -- Start Frame --
        self.frames["Start"] = ctk.CTkFrame(self.main_frame, fg_color="transparent")
        ctk.CTkLabel(self.frames["Start"], text="YOLO Workflow Dashboard", font=("Arial", 30, "bold"), text_color="gray50").pack(expand=True)

        # -- Capture Frame --
        self.frames["Capture"] = ctk.CTkFrame(self.main_frame)
        ctk.CTkLabel(self.frames["Capture"], text="Data Collection", font=("Arial", 24, "bold")).pack(pady=20)
        ctk.CTkButton(self.frames["Capture"], text="Launch Camera CaptureScript", command=lambda: self.run_script("cameraCapture.py"), 
                      height=60, width=300, font=("Arial", 18)).pack(pady=40)

        # -- Pull Images Frame --
        self.frames["Pull"] = ctk.CTkFrame(self.main_frame)
        ctk.CTkLabel(self.frames["Pull"], text="Paste Roboflow Code", font=("Arial", 24, "bold")).pack(pady=10)
        self.text_paste_code = ctk.CTkTextbox(self.frames["Pull"], height=250, width=600)
        self.text_paste_code.pack(pady=10, padx=20)
        self.text_paste_code.insert("0.0", "# Paste the code from Roboflow here...\n")
        ctk.CTkButton(self.frames["Pull"], text="Process & Download", command=self.run_pull_images, fg_color="green", height=50, width=200).pack(pady=20)

        # -- Train Frame --
        self.frames["Train"] = ctk.CTkFrame(self.main_frame)
        ctk.CTkLabel(self.frames["Train"], text="Model Training", font=("Arial", 24, "bold")).pack(pady=10)
        
        # Input: Dataset Folder
        self.entry_data_path = ctk.CTkEntry(self.frames["Train"], placeholder_text="Paste Dataset Folder Name Here (e.g. Yolo_inferencing-10)", width=400)
        self.entry_data_path.pack(pady=10)
        
        # Input: Model Name
        self.entry_train_name = ctk.CTkEntry(self.frames["Train"], placeholder_text="Name your model (e.g. NormModel)", width=400)
        self.entry_train_name.pack(pady=10)
        
        # Slider: Epochs
        self.label_epochs = ctk.CTkLabel(self.frames["Train"], text="Epoch Size: 50")
        self.label_epochs.pack(pady=(10,0))
        self.slider_epochs = ctk.CTkSlider(self.frames["Train"], from_=1, to=300, number_of_steps=299, command=lambda v: self.label_epochs.configure(text=f"Epoch Size: {int(v)}"))
        self.slider_epochs.pack(pady=10, fill="x", padx=50)
        self.slider_epochs.set(50)
        
        ctk.CTkButton(self.frames["Train"], text="Start Training", command=self.run_training, fg_color="#D32F2F", hover_color="#B71C1C").pack(pady=30)

        # -- Optimize Frame --
        self.frames["Optimize"] = ctk.CTkFrame(self.main_frame)
        ctk.CTkLabel(self.frames["Optimize"], text="Inference & Optimization", font=("Arial", 24, "bold")).pack(pady=20)
        
        # Shared Input: Target Model Name
        ctk.CTkLabel(self.frames["Optimize"], text="Target Model Name (Folder Name):").pack(pady=(10, 0))
        self.entry_opt_model_name = ctk.CTkEntry(self.frames["Optimize"], placeholder_text="e.g. NormModel", width=300)
        self.entry_opt_model_name.pack(pady=5)

        ctk.CTkButton(self.frames["Optimize"], text="1. Test Standard (.pt)", command=self.run_test_model).pack(pady=10, fill="x", padx=80)
        ctk.CTkButton(self.frames["Optimize"], text="2. Create Optimized (.engine)", command=self.run_optimizer).pack(pady=10, fill="x", padx=80)
        ctk.CTkButton(self.frames["Optimize"], text="3. Run Live Optimized", command=self.run_live_optimized, fg_color="orange", text_color="black").pack(pady=10, fill="x", padx=80)

    def show_frame(self, name):
        for frame in self.frames.values(): frame.pack_forget()
        self.frames[name].pack(fill="both", expand=True)
    
    def show_capture(self): self.show_frame("Capture")
    def show_pull(self): self.show_frame("Pull")
    def show_train(self): self.show_frame("Train")
    def show_opt(self): self.show_frame("Optimize")

    # --- Logic ---
    def check_camera_status(self):
        if os.path.exists("/dev/video0"):
            self.cam_label.configure(text="✅ Camera Connected", text_color="green")
        else:
            self.cam_label.configure(text="❌ No Camera Found", text_color="red")

    def run_script(self, script_name):
        def task():
            print(f"--- Starting {script_name} ---")
            try:
                subprocess.run(["gnome-terminal", "--", "python3", script_name], check=True)
            except FileNotFoundError:
                 subprocess.run(["python3", script_name])
        threading.Thread(target=task).start()

    def update_file_variable(self, filename, var_name, new_value, is_string=True):
        """Updates a specific variable in a python file."""
        try:
            with open(filename, 'r') as f: lines = f.readlines()
            with open(filename, 'w') as f:
                for line in lines:
                    # Match variable definition at start of line
                    if line.strip().startswith(var_name + " =") or line.strip().startswith(var_name + "="):
                        comment = ""
                        if "#" in line: comment = " #" + line.split("#")[1].strip()
                        
                        if is_string:
                            f.write(f'{var_name} = "{new_value}"{comment}\n')
                        else:
                            f.write(f'{var_name} = {new_value}{comment}\n')
                    else:
                        f.write(line)
            print(f"Updated {filename}: {var_name} -> {new_value}")
            return True
        except Exception as e:
            print(f"Error updating {filename}: {e}")
            return False

    def run_pull_images(self):
        raw_code = self.text_paste_code.get("1.0", "end")
        if "Roboflow" not in raw_code: return

        clean_lines = []
        project_name = "unknown"
        version_num = "1"

        for line in raw_code.splitlines():
            if line.strip().startswith("!pip"): continue
            
            # Extract Project Name and strip suffix (e.g. "yolo-e1txa" -> "yolo")
            if ".project(" in line:
                found = re.search(r'\.project\("([^"]+)"\)', line)
                if found: 
                    full_name = found.group(1)
                    # Split by dash, keep the first part if it looks like a generated suffix
                    if "-" in full_name:
                        project_name = full_name.rsplit("-", 1)[0] 
                    else:
                        project_name = full_name
            
            # Extract Version
            if ".version(" in line:
                found = re.search(r'\.version\(([0-9]+)\)', line)
                if found: version_num = found.group(1)

            clean_lines.append(line)

        # Save Code
        with open("postAnotation.py", "w") as f: f.write("\n".join(clean_lines))
        
        # Update Clipboard with CLEAN name
        clean_folder_name = f"{project_name}-{version_num}"
        self.clipboard_box.insert("end", f"{clean_folder_name}\n")
        self.run_script("postAnotation.py")

    def run_training(self):
        epochs = int(self.slider_epochs.get())
        data_path = self.entry_data_path.get()
        model_name = self.entry_train_name.get()

        if not model_name: model_name = "my_custom_run" # Default if empty
        
        # Add to clipboard so user remembers it
        self.clipboard_box.insert("end", f"{model_name}\n")

        self.update_file_variable("train.py", "EPOCHS", epochs, is_string=False)
        self.update_file_variable("train.py", "CUSTOM_NAME", model_name, is_string=True)
        
        if data_path:
            full_yaml_path = f"{data_path}/data.yaml"
            self.update_file_variable("train.py", "DATASET_YAML_PATH", full_yaml_path, is_string=True)
            
        self.run_script("train.py")

    def run_test_model(self):
        target_name = self.entry_opt_model_name.get()
        if not target_name: target_name = "train" # default
        
        # Update path in testModel.py
        new_path = f"runs/detect/{target_name}/weights/best.pt"
        self.update_file_variable("testModel.py", "MODEL_PATH", new_path, is_string=True)
        self.run_script("testModel.py")

    def run_optimizer(self):
        target_name = self.entry_opt_model_name.get()
        if not target_name: target_name = "train"
        
        # Update paths in modelOptimizement.py
        pt_path = f"runs/detect/{target_name}/weights/best.pt"
        self.update_file_variable("modelOptimizement.py", "MODEL_PATH", pt_path, is_string=True)
        self.run_script("modelOptimizement.py")

    def run_live_optimized(self):
        target_name = self.entry_opt_model_name.get()
        if not target_name: target_name = "train"

        # Update path in runLiveOptimized.py
        # Note: The engine file is usually saved in the same folder as the .pt file
        engine_path = f"runs/detect/{target_name}/weights/best.engine"
        self.update_file_variable("runLiveOptimized.py", "MODEL_PATH", engine_path, is_string=True)
        self.run_script("runLiveOptimized.py")

if __name__ == "__main__":
    app = YoloControlPanel()
    app.mainloop()
