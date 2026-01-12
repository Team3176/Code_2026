# Robot Code for 2026 REBUILT FRC Game & Season

## How to use this code
### WARNINGS
* ***Do NOT attempt to execute ANY command involving the elevator.  You risk mechanically breaking the robot if you do so***

### Setup and Installs (Getting started)
* Install [NI Game Tools 2026 Beta 1]( https://github.com/wpilibsuite/2026Beta/releases/tag/NI_GAME_TOOLS_BETA_1) following these [instructions](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/frc-game-tools.html)

* Install [WPILib Beta 1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.1.1-beta-1) following these [instructions](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html)

* Image your RoboRIO 2 using these [instructions](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-3/roborio2-imaging.html)

* Program your VH-109 FRC Radio using these [instruction](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-3/radio-programming.html)

* Install [Pathplanner v2025.2.2](https://github.com/mjansen4857/pathplanner/releases/tag/v2025.2.2)

* Install [CTRE Phoenix Tuner X application](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html) from this [Microsoft Store link](https://apps.microsoft.com/detail/9NVV4PWDW27Z)

* Connect the Phoenix Tuner X to the robot using these [instructions](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/connecting.html)

* Upgrade the firmware on all CTRE devices following these instructions for a ["Batch Field Upgrade"](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/device-list.html#batch-field-upgrade)

* Download the CANivore driver [here](https://ctre.download/files/canivore-usb-kernel_1.16_armv7a.ipk)

* Transfer the CANivore driver following these [instructions](https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ftp.html)

* Connect to the RoboRIO and install the CANivore driver file you just transferred, via the below procedure:
    * Open Windows powershell
    * Connect the usb cable between your computer and the RoboRIO
    * In the powershell type ```ssh lvuser@172.22.11.2```.  If asked for a password just hit the "ENTER" key.
    * Verify the CANivore driver file that you uploaded is present by typing ```ls``` and then the "ENTER" key.
    * Install the CANivore driver by typing the following command ```opkg install canivore-usb-kernel_1.16_armv7a.ipk``` and then the "ENTER" key.
    * Reboot the roboRIO (CANivore status light should now be blinking GREEN).

* Install [GitKraken Desktop](https://www.gitkraken.com/)

* Clone this repo following this [example](https://help.gitkraken.com/gitkraken-desktop/open-clone-init/#clone-a-project) but using this [repo URL](https://github.com/char3176/Code_2026)

* Setup your Driverstation following these [instructions](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-4/running-test-program.html)

* Install [Elastic](https://frc-elastic.gitbook.io/docs) from this [link to Elastic 2026.0.0 Beta 1](https://github.com/Gold872/elastic-dashboard/releases/tag/v2026.0.0-beta-1)

* Connnect your computer to the robot via either usb or wireless and then "Build & Deploy" your code using the following [example](https://docs.wpilib.org/en/latest/docs/software/vscode-overview/deploying-robot-code.html)



If you have questions or issues, I can be reached via char3176@gmail or on discord @charfxn


 

