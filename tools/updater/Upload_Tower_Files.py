import paramiko
from scp import SCPClient
import os
import sys
import ctypes
from ssh_utils import get_ssh_client, run_remote_command

def error_popup(message, title="Error"):
    """
    Shows a popup window if an error occurs.
    """
    ctypes.windll.user32.MessageBoxW(0, message, title, 0)

def upload_tower_files(key_path, local_path, remote_path):
    """
    Uploads the required project files to the Raspberry Pi.
    """
    # List required files
    required_files = [
        "main.py",
        "Tower_Class.py",
        "Driver_Class.py",
        "Encoder_Class.py",
        "Event_Class.py"
    ]

    # Check for missing files
    missing = [f for f in required_files if not os.path.isfile(os.path.join(local_path, f))]
    if missing:
        error_popup(f"Missing reuqired files: {', '.join(missing)}")
        sys.exit(1)
    
    # Start ssh session
    ssh = get_ssh_client(pi_ip="10.0.0.220", pi_username="pi", key_path=key_path)

    try:
        # Stop the service
        run_remote_command(ssh, command=f"sudo systemctl stop tower.service")

        # Transfer the files
        with SCPClient(ssh.get_transport()) as scp:
            for filename in required_files:
                local_file = os.path.join(local_path, filename)
                print(f"Uploading {local_file} to {remote_path}...")
                scp.put(local_file, remote_path)
                print(f"Done.")
        
        # Start the service
        run_remote_command(ssh, command="sudo systemctl start tower.service")
    finally:
        ssh.close()
        
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Directory that file is running form
    key_path = os.path.join(os.path.expanduser("~"), ".ssh", "id_rsa")  # SSH key path
    remote_path = "~/tower/"  # Raspberry Pi folder to upload to
    
    upload_tower_files(
        key_path = key_path,
        local_path = script_dir,
        remote_path = remote_path
    )