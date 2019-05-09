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

/** @file keyboard_reader.cpp
 * 
 *  @author karl.kruusamae(at)utexas.edu
 * 
 *  NOTE: If you get permission denied when starting this node. Use ' ls -l /dev/input/event* ' to learn which group can access the events.
 *        Then add your username to this group with ' sudo usermod -a -G group_name user_name '
 */

#include "keyboard_reader/keyboard_reader.h"


/** Goes through all the event files in /dev/input/ to locate keyboard.
 *  @return file descriptor if all checks out; 0 otherwise.
 */
int Keyboard::findKeyboard()
{
  int i, r;

  // using glob() [see: http://linux.die.net/man/3/glob ] for getting event files in /dev/input/
  glob_t gl;
  int num_event_dev = 0;					// number of relevant event files found in /dev/input/
  // use keyboard drive case1 taowenyin
  if(glob("/dev/input/by-path/platform*kbd", 0, NULL, &gl) == 0)		// looks for filenames that match the given pattern
  {
    num_event_dev = gl.gl_pathc;				// get the number of matching event files
  }
  
  printf("\x1b[1;32mHere is the list of likely candiates for your keyboard. This function always starts with the \x1b[4mfirst one on the list.\x1b[0m ");
  printf("\x1b[1;33mIf nothing happens when you press keyboard keys, terminate this process and re-start with user-specified device.\n\x1b[0m");
  
  // print all the paths that were found by glob()
  for(i=0; i<num_event_dev; ++i)
  {
    printf("[%d] %s \n",i, gl.gl_pathv[i]);
  }
  
  // try to open every path found by glob
  for(i=0; i<num_event_dev; ++i)				// go through all relevant event files
  {
    r = openKeyboard(gl.gl_pathv[i]);				// try to open an event file
    if(r >= 0)							// if openKeyboard was succesful
    {
      device_path_ = gl.gl_pathv[i];
      globfree(&gl);						// free memory allocated for globe struct
      return r;							// return descriptor
    } // if
  } // for

  globfree(&gl);					// free memory allocated for globe struct

  return 0;							// return error 0 otherwise
} // end find_keyboard


/** Opens the input device and checks whether its meaningful name (ie, EVIOCGNAME in ioctl()) contains substrings specified in valid_substring.
 *  @param device_path file name of a linux event.
 *  @return file descriptor if open and other checks have been succesfully passed; 0 otherwise.
 */
int Keyboard::openKeyboard( const char *device_path )
{
  printf("Opening device: %s \n", device_path);

  // file descriptor to the opened device. Nonblock so we can spin while waiting for data
  descriptor_ = open(device_path, O_RDONLY|O_NONBLOCK,S_IRWXU);

  // if failed to open device_path
  if(descriptor_ < 0)
  {
    fprintf(stderr, "Unable to open \"%s\": %s\n", device_path, strerror(errno));
    printf("Trying again\n");
    return 0;
  }

  /* NOTE: If your keyboard does not have substring specified in valid_substring in its EVIOCNAME, uncomment following line */
//   return descriptor_;

  // Following is a fail-safe to avoid opening non-keyboard event by checking if the input device_path has valid_substring in its name.
  char name[255];						// meaningful, ie EVIOCGNAME name
  if( ioctl(descriptor_, EVIOCGNAME( sizeof(name) ), name ) < 0) 	// fetches the meaningful (ie. EVIOCGNAME) name of the device desribed by descriptor descriptor_
  {
    fprintf(stderr, "\"%s\": EVIOCGNAME failed: %s\n", device_path, strerror(errno));
    close(descriptor_);
    return 0;
  }
  
  std::ostringstream sstream;
  sstream << name;						// convert char* to stringstream
  std::string name_as_string = sstream.str();			// stringstream to string
  int i;
  for (i=0; i < valid_substrings.size(); ++i)
  {
    std::size_t found = name_as_string.find( valid_substrings[i] );// does the meaningful name contain a predefined valid substring
    if (found!=std::string::npos) // set error case 2 taowenyin
    {
      printf("Found \x1b[1;34m'%s'\x1b[0m device. Starting to read ...\n", name);
      return descriptor_;						// if everything checks out, returns the file descriptor
    } // end if
  } // end for

  printf("%s does not seem to be a keyboard.\n", device_path);
  close(descriptor_);
  return 0;
} // end openKeyboard


/** Closes the device specificed by descriptor_. */
void Keyboard::closeKeyboard()
{
  printf("Closing keyboard device.\n");
  close(descriptor_);
  return;
}


/** Checks if the keyboard event file has been succesfully opened.
 *  @return TRUE when descriptor_ is not negative, FALSE otherwise.
 */
bool Keyboard::isReadable ()
{
  if (descriptor_ < 0) 
    return false;
  return true;
}


/** Searches the input event for type EV_KEY and returns event code (ie, which key) and value (ie, pressed or depressed).
 *  @param ev input event.
 *  @return vector containing two unsigned integers: event code (based on linux/input.h) and event value (1 for pressed, 0 for depressed); returns {0, 0} if no EV_KEY.
 */
std::vector <uint16_t> Keyboard::processEvent(struct input_event *ev)
{

  std::vector <uint16_t> event;			// output vector
  
  switch(ev->type)				// switch to a case based on the event type
  {
    case EV_SYN:				// this event is always present but no need to do anything
       //printf("EV_SYN: code=0x%04x, value=0x%08x\n", ev->code, ev->value);
      break; 
    case EV_MSC:				// this event is always present but no need to do anything
       //printf("EV_MSC: code=0x%04x, value=0x%08x\n", ev->code, ev->value);
      break;
    case EV_LED:                                // LED event occured (associated with capslock and numlock)
    case EV_KEY:				// key event means that a key was either pressed or depressed
      if (ev->value == 1)			// a key was pressed
      {
	printf("A key was pressed: code=0x%04x, value=0x%08x\n", ev->code, ev->value);
      }
      else					// a key was depressed
      {
	printf("A key was depressed: code=0x%04x, value=0x%08x\n", ev->code, ev->value);
      }
      event.push_back(ev->code);		// push event code to output vector
      event.push_back(ev->value);		// push event value to output vector
//       printf("Decimal event_code=%d \n", event[0]);
      return event;				// return output vector containing event code and value
      break;
    default:					// default case
      printf("Warning: unexpected event type; ev->type = 0x%04x\n", ev->type);
  } // end switch

  return {0, 0};				// return vector with two zeros as elements
} // end processEvent


/** Reads event data, and returns relevant info only for EV_KEY events, and dismisses anything that is not a key being pressed or depressed.
 *  @return vector containing two unsigned integers: event code (based on linux/input.h) and event value (1 for pressed, 0 for depressed).
 */
std::vector <uint16_t> Keyboard::getKeyEvent()
{
  int const BUFFER_SIZE = 64;
  struct input_event ibuffer[BUFFER_SIZE];				// see: https://www.kernel.org/doc/Documentation/input/input.txt
  int r, events, i;
  std::vector <uint16_t> event_info;					// processed event
  
  // read events to input_event buffer
  r = read(descriptor_, ibuffer, sizeof(struct input_event) * BUFFER_SIZE);
  
  if( r > 0 )
  {
      events = r / sizeof(struct input_event);				// getting the number of events

      // going through all the read events
      for(i=0; i<events; ++i)
      {
        // call processEvent() for every read event
	      event_info = processEvent(&ibuffer[i]);
	      // return only the code for events different from 0; ie, only when key was pressed or depressed
        if (event_info[0] > 0)
        {
          return event_info;
        } 
      } //end for
  }
  else
  {
    printf("read() failed: %s\n", strerror(errno));	// let user know that read() failed
    return {0, 0};
  }

} // end getKeyEvent


/** Return a string corresponding to a key code.
 *  @param key_code uint corresponding to key on a keyboard
 *  @return string corresponding to a key code
 */
std::string Keyboard::getKeyName(uint16_t key_code)
{
    return keymap_[key_code];
}


/** Grab the keyboard for this application only
*/  
bool Keyboard::grabKeyboard()
{
  ioctl(descriptor_, EVIOCGRAB, 0);
  if(ioctl(descriptor_, EVIOCGRAB, 1))
  {
    printf("Couldn't grab the keyboard. %s.\n", strerror(errno));
    return 0;
  }

  return 1;
}


/** Release the keyboard for all applications to use.
*/ 
bool Keyboard::ungrabKeyboard()
{
  if(ioctl(descriptor_, EVIOCGRAB, 0))
  {
    printf("Couldn't un-grab the keyboard. %s.\n", strerror(errno));
    return 0;
  }

  return 1;
}