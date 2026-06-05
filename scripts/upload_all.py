import subprocess
import argparse
import sys
import os
import shutil

def get_pio_command():
    """Locate the platformio executable on Windows or Linux."""
    executable = shutil.which("platformio") or shutil.which("pio")
    if executable:
        return executable
    # Common Windows path for VS Code / PlatformIO core
    home = os.path.expanduser("~")
    pio_path = os.path.join(home, ".platformio", "penv", "Scripts", "platformio.exe")
    if os.path.exists(pio_path):
        return pio_path
    return "platformio" # Fallback

def run_pio(target_name, env, port=None):
    """Runs a specific PlatformIO target."""
    pio_cmd = get_pio_command()
    cmd = [pio_cmd, "run", "-e", env, "--target", target_name]
    if port:
        cmd.extend(["--upload-port", port])
    
    print(f"\n--- Running: {' '.join(cmd)} ---")
    # shell=True is often required on Windows to resolve the pio command correctly
    result = subprocess.run(cmd, shell=True)
    return result.returncode == 0

def main():
    parser = argparse.ArgumentParser(description="DrawRobot Deployer: Firmware + Web Dashboard")
    parser.add_argument("--port", help="Specify serial port (e.g., COM3)")
    parser.add_argument("--env", default="esp32_real", help="Target environment (default: esp32_real)")
    args = parser.parse_args()

    # Change directory to project root to find platformio.ini
    current_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(current_dir)
    if not os.path.exists("platformio.ini"):
        os.chdir("..")

    if not os.path.exists("platformio.ini"):
        print("❌ Error: Could not find platformio.ini. Run this from the project root.")
        sys.exit(1)

    if not os.path.exists("data"):
        print("❌ Error: 'data' folder not found. Nothing to upload to the filesystem.")
        sys.exit(1)

    print(f"🚀 Starting DrawRobot Deployment (Env: {args.env})")

    # 1. Build and Upload Firmware
    print("\n📦 Step 1/2: Uploading C++ Firmware...")
    if not run_pio("upload", args.env, args.port):
        print("\n❌ Error: Firmware upload failed.")
        sys.exit(1)

    # 2. Build and Upload Filesystem (LittleFS)
    print("\n📂 Step 2/2: Uploading Web Dashboard (data/ folder)...")
    if not run_pio("uploadfs", args.env, args.port):
        print("\n❌ Error: Filesystem upload failed. Ensure the ESP32 is still connected.")
        sys.exit(1)

    print("\n✨ SUCCESS: Firmware and Dashboard have been deployed.")
    print("🔌 Connect to 'RobotWifi' and navigate to http://192.168.4.1")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nStopped by user.")