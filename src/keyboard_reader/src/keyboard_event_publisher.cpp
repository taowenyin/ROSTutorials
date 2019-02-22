// Copyright (c) 2015-2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/** @file keyboard_event_publisher.cpp
 * 
 *  'rosrun keyboard_reader keyboard_event_publisher'
 *  
 *  OR
 * 
 *  'rosrun keyboard_reader keyboard_event_publisher _path:=[user_specified_keyboard_path]'
 * 
 *  @author karl.kruusamae(at)utexas.edu
 */

#include "ros/ros.h"
#include "keyboard_reader/check_for_keyboard_priority.h"
#include "keyboard_reader/keyboard_reader.h"
#include "keyboard_reader/Key.h"

// Main function and a ROS publisher
int main(int argc, char *argv[])
{
  // ROS init
  ros::init(argc, argv, "keyboard_event_publisher");
  // Use async spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ROS node handle
  ros::NodeHandle nh;
  // Private node handle for optional ROS parameter _path
  ros::NodeHandle pnh("~");

  // Getting user-specified path from ROS parameter server
  std::string keyboard_path;
  pnh.param<std::string>("path", keyboard_path, "");
  
  // Notify user if no keyboard was specified
  if (keyboard_path.empty())
  {
    ROS_INFO("No keyboard specified, let's find one.");
  }

  // Create a Keyboard object
  Keyboard keyboard(keyboard_path);
  
  // If failed to open any keyboard input event, print info and exit
  if(!keyboard.isReadable())
  {
    ROS_INFO("Unable to locate keyboard.");
    ROS_INFO("Try: %s [device]\n", argv[0]);
    return 1;
  }

  // Determine whether the GUI window with focus should receive keyboard
  // commands, or if this app should.
  keyboard_priority::KeyboardPriority keyboard_priority_manager;

  // Creates publisher that advertises Key messages on rostopic /keyboard
  ros::Publisher pub_keyboard = nh.advertise<keyboard_reader::Key>("keyboard", 100);
  
  // Message for publishing key press events
  keyboard_reader::Key key_event;

  // Vector containing event data
  std::vector <uint16_t> events;

  bool keyboard_is_grabbed = false;

  while(ros::ok())
  {
    // Grab the keyboard for this application, if it hasn't been already
    if ( !keyboard_is_grabbed )
    {
      keyboard.grabKeyboard();
      keyboard_is_grabbed = true;
    }

    // Compose a publishable message
    events.clear();
    events = keyboard.getKeyEvent();
    if (events.size() > 0)
    {
        key_event.key_code = events[0];        // event code
        key_event.key_name = keyboard.getKeyName(events[0]);         // string corresponding to event code
        key_event.key_pressed = (bool)events[1];     // true when key is pressed, false otherwise
    }

    if (events[0] > 0)
    {
      pub_keyboard.publish(key_event);    // publish a Key msg only if event code is greater than zero
    }

    // Release the keyboard for other applications to use
    else
    {
      if ( keyboard_is_grabbed )
      {
        if ( keyboard.ungrabKeyboard() )
          keyboard_is_grabbed = false;
      }
    }

    // Clear for the next round
    key_event = keyboard_reader::Key();
    ros::Duration(0.01).sleep();
  } // end while
  
  keyboard.closeKeyboard();

  return 0;
} //end main