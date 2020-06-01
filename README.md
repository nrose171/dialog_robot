# dialog_robot# natalieREU

Packages Required:

    python 3
    ros *can be downloaded here: http://wiki.ros.org/ROS/Installation*
    pocketsphinx *can be downloaded here: https://github.com/UTNuclearRoboticsPublic/pocketsphinx*
    sound_play *rospackage can be downloaded here: http://wiki.ros.org/sound_play*

What needs to be changed:

    1.Need to chang the file directory strings in the dialog_robot/src/dialog.launch file *lines 4,5,6, & 12*
    2.Need to change the file directory strings in the dialog_robot/src/dialog.py file *line 18*

How to Run:

    1. roscore *runs ros master node*
    2. rosrun sound_play soundPlay_node.py *runs soundplay node*
    3. roslaunch dialog_robot dialog.launch *runs dialog_robot node*
