#include "keyboard_reader/check_for_keyboard_priority.h"

// Modified from https://github.com/UltimateHackingKeyboard/current-window-linux/blob/master/get-current-window.c
// On 8/16/2018
// Released under the GNU General Public License v3.0
// https://github.com/UltimateHackingKeyboard/current-window-linux/blob/master/LICENSE

// These aren't in the header due to preprocessor clashing.
// See https://stackoverflow.com/questions/22400905/eigen-and-cimg-compatibility-issues
#include "X11/Xatom.h"
#include "X11/Xlib.h"

#define MAXSTR 1000

// Global due to namespace clashing
Display *display;


namespace keyboard_priority
{

KeyboardPriority::KeyboardPriority()
{
  char *display_name = NULL;
  display = XOpenDisplay(display_name);
  if (display == NULL)
  {
    ROS_ERROR_STREAM("[check_for_keyboard_priority] Unable to open X11 display.");
    ros::shutdown();
    exit(1);
  }
  screen_ = XDefaultScreen(display);  
}

KeyboardPriority::~KeyboardPriority()
{
  XCloseDisplay(display);
}

// return true if the active GUI window is on the whitelist
bool KeyboardPriority::checkForKeyboardPriority()
{
  window_ = RootWindow(display, screen_);
  window_ = getLongProperty("_NET_ACTIVE_WINDOW");

  // Compare the name of the active window to the white list
  std::string window_with_focus(reinterpret_cast<char*>( getStringProperty("WM_CLASS") ));
  for (int i=0; i<keyboard_whitelist_.size(); ++i)
  {
    if ( window_with_focus.find( keyboard_whitelist_[i] ) != std::string::npos )
      return true;
  }
  // Otherwise this app should ignore the keyboard commands

  return false;
}

void KeyboardPriority::checkStatus(int status, unsigned long window)
{
    if (status == BadWindow) {
        ROS_ERROR_STREAM("[check_for_keyboard_priority] Window id does not exist!");
        ros::shutdown();
        exit(1);
    }

    if (status != Success) {
        ROS_ERROR_STREAM("XGetWindowProperty failed!");
        ros::shutdown();
        exit(2);
    }
}

unsigned char* KeyboardPriority::getStringProperty(char* property_name)
{
    Atom actual_type, filter_atom;
    int actual_format, status;
    unsigned long nitems, bytes_after;
    unsigned char *prop;

    filter_atom = XInternAtom(display, property_name, True);
    status = XGetWindowProperty(display, window_, filter_atom, 0, MAXSTR, False, AnyPropertyType,
                                &actual_type, &actual_format, &nitems, &bytes_after, &prop);
    checkStatus(status, window_);
    return prop;
}

unsigned long KeyboardPriority::getLongProperty(char* property_name)
{
    unsigned char *prop = getStringProperty(property_name);
    unsigned long long_property = prop[0] + (prop[1]<<8) + (prop[2]<<16) + (prop[3]<<24);
    return long_property;
}

} // End namespace keyboard_priority