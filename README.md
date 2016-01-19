# GBoxController

Design for a 31 button USB Windows game controller built using an ATMega328 microcontroller for use with Flight Simulators.

This software was built using the following tools and libraries

<a href="https://www.obdev.at/products/vusb/index.html">V-USB</a><br/>
<a href="http://www.atmel.com/tools/STUDIOARCHIVE.aspx">Atmel Studio 6.2</a><br/>
<a href="https://www.arduino.cc/en/Main/Software">Arduino IDE</a><br/>
<a href="https://code.google.com/p/diy-layout-creator/">DIY Layout Creator</a><br/>
<a href="http://fritzing.org/home/">Fritzing</a><br/>

The device utilises the built in Windows human interface device (HID) driver and should appear in the Control Panel under Game Controllers as a 31 button device.  The controller has:
<ul><li>8 x two way switches (one button each)
<li>4 x three way switches (two buttons each)
<li>5 x rotary encoders with push buttons (three buttons each)</ul>

See the schematics folder for logical and physical circuit diagrams.  I have included png renderings of these diagrams in case you don't have/want the relevant software to view them.

In order to load onto the ATMega chip, I used the Arduino IDE with an Arduino Uno to act as the in service programmer (ISP).  See <a href="https://www.arduino.cc/en/Tutorial/ArduinoISP">this tutorial</a> for an example of how to do this.  The circuit design contains a standard 6 pin ISP socket for this purpose.

The Atmel Studio solution will compile to a binary .hex file.  I found it necessary to modify the Arduino IDE's boards.txt to include an additional entry entitled 'ATMega' which pointed to the location of the compiled binary file in order to use the Arduino ISP.

The controller is designed to send a pulse (button press) each time the state of a switch changes. For example, moving a two way switch up, down, up will result in three separate button presses for the corresponding button.
