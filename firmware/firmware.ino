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
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);
  delay(100);
  digitalWrite(13, 0);
  delay(100);
  digitalWrite(13, 1);
  delay(100);
  digitalWrite(13, 0);
  delay(100);


  Serial.begin(9600);

  s_l.setSpeed(50);
  s_r.setSpeed(50);

  Serial.print("Muralizer v" );
  print_version();
  print_help();
}

void spin_bresenham(int dr0, int dr1) {
  int which_is_x = 0;
  int dx = abs(dr0);
  int dy = abs(dr1);

  // ensure we always take enough steps
  if (dy > dx) {
    which_is_x = 1;
    dx = abs(dr1);
    dy = abs(dr0);
  }

  // constants for quadrant fixups
  int s0 = dr0 < 0 ? -1 : 1;
  int s1 = dr1 < 1 ? -1 : 1;

  int D = 2*dy - dx;

  for (int x = 0; x < dx; x++) {
    if (which_is_x) 
      s_r.step(s1*1);
    else
      s_l.step(s0*1);

    if (D > 0) {
      if (which_is_x)
	s_l.step(s0*1);
      else
	s_r.step(s1*1);

      D -= 2*dx;
    }

    D += 2*dy;
    delay(1);
  } // for x
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
      memset(buf, 0, BUFLEN);
      return;
    }

    if (x == '\n') {
      switch(buf[0]) {
      case 'v':
	print_version();
	memset(buf, 0, BUFLEN);
	break;
      case 'r':
	s0 = strchr(buf, ' ');
	if (NULL == s0) {
	  Serial.println("Bogus input line, s0.");
	  memset(buf, 0, BUFLEN);
	  return;
	}

	s1 = strchr(s0+1, ' ');
	if (NULL == s0) {
	  Serial.println("Bogus input line, s1.");
	  memset(buf, 0, BUFLEN);
	  return;
	}

	i0 = atoi(s0);
	i1 = atoi(s1);


	memset(buf, 0, BUFLEN);

	Serial.print("Rotating ");
	Serial.print(i0);
	Serial.print(" ");
	Serial.println(i1);
#if 0	  
	if (i0 != 0)
	  s_l.step(i0);

	if (i1 != 0)
	  s_r.step(i1);
#else
	spin_bresenham(i0, i1);
#endif

	break;
      default:
	Serial.print("Unknown command. ");
	print_help();
	memset(buf, 0, BUFLEN);
	break;
      }
      cursor = 0;
    }
    
  }
}
