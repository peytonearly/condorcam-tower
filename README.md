* What the project is
* Folder map (pi_runtime/firmware/tools/tests)
* How to run on the Pi for now
Note that systemd deploy lives under deploy/systemd

## Project Description
Github repo containing code for the Condorcam Tower.

## Repo Structure
Pico firmware:
> firmware\pico\

Pi runtime scripts:
> pi_runtime\

Test scripts:
> tests\scripts\

Updater and log downloader scripts:
> tools\updater\

Python libraries required for operation:
> requirements.txt

## Setup Instructions
Clone repo:
> git clone https://github.com/peytonearly/condorcam-tower
> cd condorcam-tower/deploy
> chmod +x bootstrap_pi.sh install_service.sh
> ./bootstrap_pi.sh
> ./install_service.sh