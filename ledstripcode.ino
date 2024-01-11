/* Hello future me, or other people interested in Adafruit DotStar's led library.
      Setup 
 To setup an Arduino (uno) to correctly use LED strips, know this.
 - Every strip has 2 Digital Input (DI) pins. set them accordingly.
 - Include the DotStar Library.
 - #define NUMPIXELS, DATAPIN, and CLOCKPIN. the "PIN" variables correspond to the DI pins you set.
 - then, depending on if you want to use a full strip or a small number of pixels:
 
 IF using 20 Pixels or Higher:
 Use a power supply. Preferrably a battery into a bus with multiple outputs.
 Plug in the thicker wires of the strip into the - and + 5V inputs. USE 5 VOLTS. IF HIGHER OR LOWER THINGS MAY GO KAPOOT.
 be sure to plug in the smaller 5V and GND wire into the ardino.  

 IF not:
 plug in the 5V wire into the 5V input, and GND to any GND input. easy as pie.

        Pixel Commands
 strip.show(); - the most important of the other commands. shows any changes made once and once only.
 strip.setPixelColor(pixel, r,g,b); "pixel" is the specific pixel you want to change the color for. use ints if you want to spice them up!
 strip.clear(); self explanatory. clears the strip changes.
 strip.begin(); Initalizes the strip to be ready for use.
 


*/
#include <Adafruit_DotStar.h>
#define NUMPIXELS 120// Number of LEDs in strip
#define DATAPIN    4
#define CLOCKPIN   5
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR); /* Sometimes your led strip doesnt have the right colors. switch the RGB letters around and itll work. */

int led = 0;
int maxled = NUMPIXELS;
int led2 = 0;
bool ledswitch = false;
void setup() {
  // put your setup code here, to run once:
strip.begin();
strip.clear();
strip.show();
led2 = 16;
}

void loop() {
  // put your main code here, to run repeatedly:
strip.setPixelColor(led, 50, 0, 55);
led++;
strip.setPixelColor(led2, 0, 0, 0);
led2++;
delay(0.5);
strip.show();
if (led >= maxled) { 
  led2 = 16;
  led = 0;
}
}
