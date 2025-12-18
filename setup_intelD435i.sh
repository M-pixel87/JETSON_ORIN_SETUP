#!/bin/bash

# =================================================================
# Automated Installer for Intel RealSense D435i on Jetson Orin AGX
# Target System: JetPack 6 (Ubuntu 22.04)
# Based on JetsonHacks & Intel RealSense repositories
# =================================================================

# Stop the script if any command fails
set -e

echo "Starting RealSense D435i Installation..."
echo "-------------------------------------------------"

# --- Phase 1: Download and Patch Kernel ---
echo "[Step 1/5] Cloning JetsonHacks repository..."
if [ -d "jetson-orin-librealsense" ]; then
    echo "Directory 'jetson-orin-librealsense' already exists. Removing it to ensure a clean install..."
    rm -rf jetson-orin-librealsense
fi
git clone https://github.com/jetsonhacks/jetson-orin-librealsense.git
cd jetson-orin-librealsense

echo "[Step 2/5] Installing custom Kernel Modules (The 'Fix')..."
# Extract the modules archive
tar -xvzf install-modules.tar.gz
cd install-modules
# Run the install script with sudo
sudo ./install-realsense-modules.sh
# Go back to the original directory
cd ../..

# --- Phase 2: Install RealSense SDK ---
echo "[Step 3/5] Registering Intel Public Keys..."
sudo mkdir -p /etc/apt/keyrings
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

echo "[Step 4/5] Adding Intel Repository and Installing Libraries..."
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get update
# -y flag automatically answers "yes" to prompts
sudo apt-get install -y librealsense2-utils librealsense2-dev

# --- Phase 3: Finalize ---
echo "[Step 5/5] Reloading udev rules..."
# Fixed the permission error by adding sudo to the trigger command
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "-------------------------------------------------"
echo "Installation Complete!"
echo "Please REBOOT your Jetson Orin now to finalize the changes."
echo "After reboot, run 'realsense-viewer' to test."
