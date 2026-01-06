import paramiko
from scp import SCPClient

def get_ssh_client(pi_ip, pi_username, key_path):
    """
    Connects to the Raspberry Pi via ssh.
    """
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(pi_ip, username=pi_username, key_filename=key_path)
    return ssh

def run_remote_command(ssh, command):
    """
    Sends a remote command to the Raspberry Pi.
    """
    print(f"Running: {command}")
    stdin, stdout, stderr = ssh.exec_command(command)
    exit_status = stdout.channel.recv_exit_status()
    if exit_status != 0:
        error_msg = stderr.read().decode().strip()
        raise RuntimeError(f"Command failed: {command}\n{error_msg}")