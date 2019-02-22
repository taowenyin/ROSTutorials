#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include "string"
#include "vector"

// Work with X11 to determine whether this app should have access to keyboard commands.
// There is a list of other applications that take priority when they are the
// window in focus.

#ifndef CHECK_FOR_KEYBOARD_PRIORITY_H
#define CHECK_FOR_KEYBOARD_PRIORITY_H

namespace keyboard_priority
{

  class KeyboardPriority
  {
  public:
		KeyboardPriority();
		~KeyboardPriority();

    bool checkForKeyboardPriority();

  private:
    // Check whether the given window exists
    void checkStatus(int status, unsigned long window);

    // Get a string property from an X11 window
    unsigned char* getStringProperty(char* property_name);

    // Format longer X11 property strings
    unsigned long getLongProperty(char* property_name);

    // Define any GUI windows that can pass keyboard commands when
    // they have focus
    std::vector<std::string> keyboard_whitelist_{"rviz"};

    unsigned long window_;
    int screen_;
  };

} // End namespace keyboard_priority
#endif