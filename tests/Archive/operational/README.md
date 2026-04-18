# Summary
The scripts contained in this folder (`condorcam-tower\tests\operational`) will be used for operational testing of the fully assembled system. The scripts contained in this folder each test an individual part of the system to ensure functionality. Once all tests have been completed, the full system test can be run with high confidence.

## Wiring the System
Refer to the two .png's for expected connections. Note that the Pi4B_Connections.png is slightly out-of-date: GPIO's 17, 22, 26, and 27 are not connected in the final setup. The image also does not depict Pwr, Gnd, and USB connections from the Pi4B - use the USB 3.0 ports (the blue ones) to connect to the Pico and Driver. In theory, it shouldn't matter what the individual ports are connected to.... It may be helpful to use a breadboard for the power and ground lines.

When initially powering the Pi, ensure that either the motor or the driver are unpowered. If the script auto-starts, it may try to drive the motor prematurely.

## Connecting to the Pi
Connect your computer to the Pi via Ethernet. Configure your network adapter to have an IPv4 of `10.0.0.2` to properly communicate with the Pi. Using Command Prompt or PowerShell, you can test connectivity to the Pi with `ping raspberrypi.local`. The Pi usually takes 1-2 minutes to load SSH after powering on, and should return your pings when ready.

Use `ssh pi@raspberrypi.local` to connect to the Pi, password `admin`. After the initial connection, the Pi should save your key/fingerprint and shouldn't need a password on future connections.

## Disabling Automatic Programs
After SSH'ing into the Pi, enter the following command to check the status of the automatic programs:
`sudo systemd status pigpiod tower.service`

This command checks the `pigpiod` and `tower.service` programs and reports the status for both. You want the "active status" of pigpiod to be "active" (green), and tower.service to be "inactive" (red). If this is not the case, run the following commands as necessary:
`sudo systemd start pigpiod` --> Starts the pigpiod service (this service connects to the GPIO pins on the Pi).
`sudo systemd stop tower.service` --> Stops the tower.service (this service automatically runs our Python scripts).

## How to Upload Files
Scripts currently live on the Pi at `~/tower/` (once SSH'ed in, you can navigate to this folder with `cd ~/tower/`). To display a list of all files in that folder, enter `ls`. 

To upload a file from your computer to the Pi, open a PowerShell or Command Prompt script from the folder that houses the desired file. Then enter the following command:
`scp <filename> pi@raspberrypi.local:~/tower/`

## How to Download Log Files
To download log files, enter the following command from your laptop's terminal (NOT the terminal that is SSH'ed into the Pi):
`scp -r pi@raspberrypi.local:~/tower/logs/ .\path\to\local\save\folder`

This command recursively downloads all logs from the Pi (can be up 5 files) to the location specified by .\path\to\local\save\folder

## Test Scripts
The sections below outline the indivudal test scripts included in this folder, as well as expected outputs. These scripts each test a specific part of the system, so it is recommended to run through all of these scripts (in the order they are listed) before running the full program.

NOTE: These scripts are new and have not been uploaded to the Pi yet. I recommend uploading them as you step through each test.