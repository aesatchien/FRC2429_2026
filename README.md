![Build Status](https://github.com/aesatchien/FRC2429_2026/actions/workflows/build.yml/badge.svg)
## FRC2429_2026

### Code repo for 2026 using robotpy
FRC robot code for the 2026 robot using robotpy-commands-v2.  

* Note code will be updated often during Jan-April 2026 build season

#### Organization
* comp_bot - 2026 robot code - TBD
* gui  - 2026 version of the python dashboard(s)
* resources
   * sim  - utilities for the simulation (field images, etc)
   * plots - useful graphs for understanding the game
* other_robots  - code for practice bots, characterization, etc
   * chairbot - our STEAM outreach go-kart
   * practicebot - stripped-down swerve base with advanced simulation capabilities
   * tankbot - stripped-down WCD base, now with a shooter and a turret
   


---
#### Where's the other stuff?
* training notebooks are here: https://github.com/aesatchien/FRC_training
* vision system (image processing on the pi) is here:  https://github.com/aesatchien/FRC2429_vision

#### Installation
Clone the git and install on your own machine:
Use "git clone https://github.com/aesatchien/FRC2429_2026.git" from the git bash (or any git aware) shell to download.  If you don't have git and you just want to look at the code, you can download the repository from the links on the right.

Need some help with git?   [Read our Team Git Rules here](./resources/git_guidelines.md)

Notes on how to install python and the necessary accessories (particularly the robotpy libraries) that will get all of this running: (TODO: UPDATE THIS DOCUMENT)
[2026 Python Installation](https://docs.google.com/document/d/17hmov_FAXwbJdad-m56ibuYFLzUg6PLvJjhuGWcnYeQ/edit?usp=sharing) but may be a bit cryptic.  I'll help if you need it.
basically, in your python environment (you may want more than one) you'll need a `pip install robotpy` and if you are working on anything else several more packages.

Once everything is installed, you need to go to the folder with robot.py.  From there, commands like (new for 2024)

```python -m robotpy sim```  

should bring up the simulator and allow you to check to see if your gamepad is recognized (you can also use the keyboard) and should be able to let you drive a virtual robot around the field if you have a gamepad. 
```python -m robotpy deploy``` and ```python -m robotpy sync``` will send your code to the robot and sync your packages, respectively.
[Read more about it here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html).

