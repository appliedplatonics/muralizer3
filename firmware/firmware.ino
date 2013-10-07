/* Local Variables:  */
/* mode: c++         */
/* comment-column: 0 */
/* End:              */


/* Muralizer 3.  Yes, three.

   Copyright (c) 2013 Applied Platonics, LLC

 */

#include <string.h>
#include <Stepper.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 1

Stepper s_l(48, 2,3,4,5);
Stepper s_r(48, 6,7,8,9);

void print_version() {
  Serial.print((int) VERSION_MAJOR);
  Serial.print(".");
  Serial.println((int) VERSION_MINOR);
}

void print_help() {
  Serial.println("Commands: [v]ersion, [r]otate <l> <r>");
}

void setup() {
  Serial.begin(9600);

  s_l.setSpeed(50);
  s_r.setSpeed(50);

  Serial.print("Muralizer v" );
  print_version();
  print_help();
}


#define BUFLEN 32
char buf[BUFLEN];
uint8_t cursor = 0;

void loop() {
  const char *s0, *s1;
  int i0, i1;


  if (Serial.available() > 0) {
    uint8_t x = Serial.read();
    buf[cursor] = x;
    cursor++;

    if (cursor == BUFLEN) {
      Serial.println("BUFFER OVERFLOW, DON'T BE THE MICROMACHINES GUY.");
      cursor = 0;
      return;
    }

    if (x == '\n') {
      switch(buf[0]) {
      case 'v':
	print_version();
	break;
      case 'r':
	s0 = strchr(buf, ' ');
	if (NULL == s0) {
	  Serial.println("Bogus input line, s0.");
	  return;
	}

	s1 = strchr(s0+1, ' ');
	if (NULL == s0) {
	  Serial.println("Bogus input line, s1.");
	  return;
	}

	i0 = atoi(s0);
	i1 = atoi(s1);

	Serial.print("Rotating ");
	Serial.print(i0);
	Serial.print(" ");
	Serial.println(i1);
	  
	s_l.step(i0);
	s_r.step(i1);


	break;
      default:
	Serial.print("Unknown command. ");
	print_help();
	break;
      }
      cursor = 0;
    }
    
  }
}
