import paramiko
from scp import SCPClient
import os
from datetime import datetime
from CondorCam.Tower.Tower_Code.Updater.ssh_utils import get_ssh_client, run_remote_command

def get_log_files_from_pi(key_path, local_path, remote_path):
    """
    Downloads the log files from the Raspberry Pi.
    """
    # Ensure local path exists
    os.makedirs(local_path, exist_ok=True)
    
    # Start ssh session
    ssh = get_ssh_client(pi_ip="10.0.0.220", pi_username="pi", key_path=key_path)
    
    try:
        # Stop the service
        run_remote_command(ssh, command=f"sudo systemctl stop tower.service")
        
        # Transfer the files
        with SCPClient(ssh.get_transport()) as scp:
            print(f"Downloading logs: {remote_path} -> {local_path}")
            scp.get(remote_path, local_path=local_path, recursive=True)
            print(f"Download complete.")
        
        # Start the service
        run_remote_command(ssh, command=f"sudo systemctl start tower.service")
        
    finally:
        ssh.close()
        
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Directory that file is being run from
    key_path = os.path.join(os.path.expanduser("~"), ".ssh", "id_rsa")  # SSH key path
    remote_path = "~/tower/logs/"  # Path to the logs file (on the Raspberry Pi)
    date_str = datetime.now().strftime("%Y-%m-%d")
    local_path = os.path.join(script_dir, f"{date_str}_logs")
    
    get_log_files_from_pi(
        key_path = key_path,
        local_path = local_path,
        remote_path = remote_path
    )