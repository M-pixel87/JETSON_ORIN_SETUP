_ ______ _____ ____   ____  _   _     ___  _____  _____ _   _ 
     | |  ____|_   _/ ___| / __ \| \ | |   / _ \|  __ \|_   _| \ | |
     | | |__    | | \___ \| |  | |  \| |  | | | | |__) | | | |  \| |
 _   | |  __|   | |  ___) | |  | | . ` |  | | | |  _  /  | | | . ` |
| |__| | |____  | | |____/| |__| | |\  |  | |_| | | \ \ _| |_| |\  |
 \____/|______| |_|_____/  \____/|_| \_|   \___/|_|  \_\_____|_| \_|
                                                                    
======================================================================
AUTOMATED SETUP & INFERENCE WORKFLOW
======================================================================

This repository contains a complete toolkit for setting up a Jetson 
AGX Orin from scratch and running high-performance YOLOv8 object 
detection.

It is designed to take a fresh Jetson (with just Wi-Fi connected) 
and turn it into a fully configured AI development machine in about 
20 minutes.

----------------------------------------------------------------------
[1] QUICK START (AUTOMATED SETUP)
----------------------------------------------------------------------
This one script installs VS Code, PyTorch 2.0, Torchvision, NoMachine, 
and sets up GPIO/Hardware permissions.

    1. Open your terminal in this folder.
    
    2. Make the script executable:
       chmod +x setup_jetson.sh

    3. Run the installer:
       ./setup_jetson.sh

    (You will need to enter your sudo password once at the start. 
     The rest is fully automated.)

----------------------------------------------------------------------
[2] PROJECT STRUCTURE
----------------------------------------------------------------------
Here is where everything lives in this repo:

  > setup_jetson.sh
    - START HERE. The master automation script for fresh Jetsons.

  > E_YOLO-Inference/
    - THE MAIN APPLICATION. Contains your custom YOLOv8 Control 
      Center App.
      L__ A_interfaceForYolo.py (Run this to launch the dashboard)

  > A_YOLOsetup.txt
    - Detailed manual guide for the "Modern" YOLO workflow.

  > B_LEGACYsetup.txt
    - Manual guide for the older jetson-inference / SSD workflow.

  > C_ARU_afall25.py/
    - Project files for the ARU Fall 2025 unit.

  > D_Milian_cspring25/
    - Project files for the Milian Spring 2025 unit.

----------------------------------------------------------------------
[3] HOW TO RUN THE YOLO APP
----------------------------------------------------------------------
Once the setup script finishes and you have rebooted, you can launch 
the Control Center:

    1. Navigate to the inference folder:
       cd E_YOLO-Inference

    2. Launch the dashboard:
       python3 A_interfaceForYolo.py

    APP FEATURES:
      * Capture:  Collect training images from the camera.
      * Pull:     Download annotated datasets directly from Roboflow.
      * Train:    Train custom YOLOv8 models on the Jetson GPU.
      * Optimize: Convert models to TensorRT (.engine) for max FPS.

----------------------------------------------------------------------
[!] NOTES
----------------------------------------------------------------------
  * REBOOT REQUIRED: After running setup_jetson.sh, you MUST reboot 
    for the GPIO and Docker permissions to take effect.

  * WI-FI: This repo assumes you have already connected to the 
    UAFS Wi-Fi manually (see A_YOLOsetup.txt Part 1.1 if you need 
    help with settings).
