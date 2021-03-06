![Alopex logo](AlopexLogo.png "Alopex Logo")
# ALOPEX TEUMESIOS

### Project Description
Alopex Teumesios is a counter-counter-UAS (CCUAS) project under development for the Wright State University course CEG 4980/81: Team Projects I/II. The goal of the project is to develop a user-piloted UAS implementing autonomous detection and avoidance capabilities to evade stationary and projectile threats with full 360&deg; coverage.

### Alopex Team
* Daniel Franklin, B.S. Computer Science
* Cody Palmer, B.S. Electrical Engineering
* Jacob Routzohn, B.S. Electrical Engineering
* David Sutherin, B.S. Computer Science

### Dependencies
* ArduCopter: [wiki](http://ardupilot.org/copter/docs/introduction.html)

### Installation
* Clone the repository
  * `git clone https://github.com/drsutherin/alopex-teumesios.git`
* cd into the directory
  * `cd alopex-teumesios`
* Initialize and update submodules
  * `git submodule update --init ardupilot/ adam/HB100_test`
* Open ADAM code in [Arduino IDE](https://www.arduino.cc/en/Main/Software) and upload
* Flash ArduPilot code to SD card and insert into Pixhawk
* Rock 'n roll!

### Acknowledgements
* ADAM microwave code based on projects by [3zuli](https://github.com/3zuli/HB100_test) and [kd8bxp](https://www.gitbook.com/book/kd8bxp/arduino-project-doppler-radar-speed-detection-usi/details)
* Alopex logo from [ClipartFest.com](https://clipartfest.com/download/b4f90b1738b49e5a93beff7a8e6bf46830dc337b.html)
