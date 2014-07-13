/* Local Variables:  */
/* mode: c++         */
/* comment-column: 0 */
/* End:              */


/* Muralizer 3.  Yes, three.

   Copyright (c) 2013,2014 Applied Platonics, LLC

   Released under the GPLv2 (and v2 only).

   This code is fairly trivial; the most complicated thing is a
   bog-standard Bresenham algorithm.  Which sounds complicated, but
   mostly means "make the motors take turns reeling in, to keep the
   pen wiggling along the line as closely as possible."

   We choose to keep this code very minimalist because writing quality
   numeric-control code on an AVR-class processor is hard and
   time-consuming to iterate.  Putting the complicated bits into the
   host controller makes the microcontroller code trivial, and makes
   it much friendlier to people experimenting with control methods.

   Happy hacking! -jbm
 */

#include <string.h>
#include <Stepper.h>

#include <Servo.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 1

Stepper s_l(48, 2,3,4,5); // Servo at top left of canvas
Stepper s_r(48, 6,7,8,9); // Servo at top right of canvas

Servo pen_servo; // on pin 11

// NB: We attach/detach from the pen servo a lot, to avoid obnoxious
// whining sounds.  This will be problematic if your pen servo's
// gearing is smooth enough to not hold itself up without power
// applied.  Handily, cheapo servos aren't nice enough for this issue.






void print_version() {
  Serial.print((int) VERSION_MAJOR);
  Serial.print(".");
  Serial.println((int) VERSION_MINOR);
}

void print_help() {
  Serial.println("Commands: [v]ersion, [r]otate <l> <r>, [p]en <u> <d>");
}


void pen_up() {
  pen_servo.attach(11);
  pen_servo.write(30);
  delay(1000);
  pen_servo.detach();
}

void pen_down() {
  pen_servo.attach(11);
  pen_servo.write(120);
  delay(1000);
  pen_servo.detach();
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

  //  pen_servo.attach(11);
  pen_up();

  Serial.begin(9600);

  s_l.setSpeed(50);
  s_r.setSpeed(50);

  Serial.print("Muralizer v" );
  print_version();
  print_help();
}



void spin_bresenham(int dr0, int dr1) {

  // XXX This is done in the wrong coordinate space.  It should be
  // choosing the next step based on which of the two final points are
  // closest to the line.

  // TODO Rewrite this with a custom stepper driver that allows
  // simultaneous pulls on both motors.

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
//    delay(1);
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

      case 'p':
	if (buf[2] == 'u') {
	  pen_up();
	  Serial.println("Pen up");
	} else if (buf[2] == 'd') {
	  pen_down();
	  Serial.println("Pen down");
	}
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
