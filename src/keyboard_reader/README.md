# keyboard_reader

Reads keyboard keys being pressed on an linux event level and publishes Key messages that contain key code, key name, and whether the key was pressed or depressed.

#### Instructions
Run:

`rosrun keyboard_reader keyboard_event_publisher`

or for user-specified keyboard:

`rosrun keyboard_reader keyboard_event_publisher _path:=<keyboard_event_file_path>`

You can modify the publishing of keyboard events based on what GUI window is active. Add your application name to the list in check_for_keyboard_priority.h.

#### Troubleshooting
**NB!** Accessing linux events requires root privileges. Make sure you have proper access, otherwise this package will not work.

Use `ls -l /dev/input/event*` to learn which group can access the events.

Then add your username to this group with `sudo usermod -a -G <group_name> <user_name>` or `sudo adduser <user_name> <group_name>`, log out and log in. For example, `sudo adduser me input`

To make it work in most cases, use the following command to add your username to the group `root`, log out, and log in.

`sudo usermod -a -G root <user_name>`
