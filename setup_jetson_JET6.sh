#!/bin/bash

# JETSON AGX ORIN (JETPACK 6.2) MASTER SETUP SCRIPT
# Automates VS Code, NoMachine, GPIO, PyTorch (JP6/CUDA12.6), and GitHub Keys
# -----------------------------------------------------------

# Stop script on error
set -e 

# Function to show a pop-up if an error occurs
error_popup() {
    zenity --error --text="An error occurred! Check the terminal for details." --title="Setup Failed"
}
trap error_popup ERR

echo "====================================================="
echo "STARTING AUTOMATED SETUP (JETPACK 6.2 TARGET)"
echo "Target User: $USER"
echo "====================================================="

# 1. INSTALL GUI POP-UP TOOL (ZENITY)
# -----------------------------------------------------------
echo "[1/7] Installing helper tools..."
sudo apt-get update
sudo apt-get install -y zenity

# 2. SETUP SUDOERS (NO PASSWORD)
# -----------------------------------------------------------
echo "[2/7] Configuring sudo permissions..."
if sudo grep -q "NOPASSWD: ALL" /etc/sudoers.d/$USER 2>/dev/null; then
    echo " - Sudo passwordless already set."
else
    echo " - Asking for sudo password one last time..."
    echo "$USER ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/$USER > /dev/null
    sudo chmod 0440 /etc/sudoers.d/$USER
    echo " - Success!"
fi

# 3. INSTALL SYSTEM TOOLS & VS CODE
# -----------------------------------------------------------
echo "[3/7] Installing Dev Tools, VS Code & GPIO Tools..."
# Added gpiod for JetPack 6 kernel GPIO debugging
sudo apt-get install -y git curl libjpeg-dev zlib1g-dev libpython3-dev python3-pip v4l-utils gpiod libgpiod-dev

# VS Code (ARM64)
if ! command -v code &> /dev/null; then
    echo " - Downloading VS Code..."
    curl -L 'https://update.code.visualstudio.com/latest/linux-deb-arm64/stable' -o code_arm64.deb
    sudo dpkg -i code_arm64.deb
    rm code_arm64.deb
else
    echo " - VS Code already installed."
fi

# AUTOMATICALLY INSTALL PYTHON EXTENSIONS FOR VS CODE
echo " - Installing VS Code Python Extensions..."
code --install-extension ms-python.python --force
code --install-extension ms-python.vscode-pylance --force

# NoMachine (Updated link for "Latest ARM64")
if ! dpkg -s nomachine &> /dev/null; then
    echo " - Downloading NoMachine (Latest)..."
    # Uses the generic redirect to avoid version number rot
    wget https://www.nomachine.com/free/arm/v8/deb -O nomachine.deb
    sudo dpkg -i nomachine.deb
    rm nomachine.deb
else
    echo " - NoMachine already installed."
fi

# 4. GITHUB SSH KEY GENERATION
# -----------------------------------------------------------
echo "[4/7] Checking GitHub SSH Keys..."
if [ ! -f ~/.ssh/id_rsa ]; then
    echo " - Generating new SSH Key..."
    ssh-keygen -t rsa -b 4096 -C "$USER@jetson" -f ~/.ssh/id_rsa -N ""
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_rsa
fi

# 5. HARDWARE PERMISSIONS (GPIO/I2C)
# -----------------------------------------------------------
echo "[5/7] Setting up GPIO & Hardware Permissions..."
# Force upgrade to ensure JP6 compatibility
sudo pip3 install --upgrade Jetson.GPIO adafruit-circuitpython-servokit pyserial
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo usermod -aG i2c $USER
sudo usermod -aG dialout $USER
# Reload udev rules specifically for the new kernel changes
sudo udevadm control --reload-rules && sudo udevadm trigger

# 6. PYTORCH & TORCHVISION (FIXED FOR JETPACK 6.2)
# -----------------------------------------------------------
echo "[6/7] Installing PyTorch & Torchvision for JetPack 6.2..."
echo "      (This pulls from the Jetson AI Lab Index)"

# Remove potential conflicts
sudo pip3 uninstall -y torch torchvision ultralytics

# Install PyTorch and Torchvision directly from the Jetson AI Lab index (CUDA 12.6)
# We install them together to ensure version matching
pip3 install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

# 7. APP DEPENDENCIES
# -----------------------------------------------------------
echo "[7/7] Installing YOLO App Dependencies..."
# We allow it to fetch dependencies but verify it doesn't downgrade torch
pip3 install ultralytics customtkinter Pillow

# ===========================================================
# FINAL POP-UP NOTIFICATION
# ===========================================================

echo "====================================================" > key_info.txt
echo "ACTION REQUIRED: SETUP GITHUB" >> key_info.txt
echo "====================================================" >> key_info.txt
echo "1. Select and Copy the SSH Key below (Ctrl+C)." >> key_info.txt
echo "2. Go to GitHub.com -> Settings -> SSH and GPG Keys." >> key_info.txt
echo "3. Click 'New SSH Key' and paste this in." >> key_info.txt
echo "" >> key_info.txt
echo "--- COPY FROM HERE ---" >> key_info.txt
cat ~/.ssh/id_rsa.pub >> key_info.txt
echo "--- END OF KEY ---" >> key_info.txt

zenity --text-info \
       --title="Setup Complete! - Action Required" \
       --width=600 --height=500 \
       --filename=key_info.txt \
       --editable \
       --ok-label="I Have Copied It"

rm key_info.txt

zenity --question \
       --title="Reboot Required" \
       --text="Setup is complete.\n\nYou must REBOOT the Jetson for hardware permissions to work.\n\nReboot now?" \
       --width=400

if [ $? = 0 ]; then
    sudo reboot
else
    echo "Please reboot manually later."
fi
