## Schematic

### Theory of Operation

### Display LEDs

These are multiplexed with a single IO pin driving the anode of each pixel and the RGB cathodes connected to timers to PWM them (blue is slightly different, see booster below). 

The software lights each pixel in turn after enough that you can't see it. You could potentially also light multiple pixels at the same time if that hard the same color, but ultimately you will hit the maximum current rating for the IO pins so probably not worth it. You might be able to drive one or 2 pixels statically if you wanted it to look really bright, but be careful not to blow them!  

Q: Why not put all the anode drivers on a single `PORT` in sequence so we can just walk a bit rather than needing a look-up for which port/bit is next?

A: This would have been really elegant, but I Really wanted to be able to drive the RGB lines with timers for efficiency so PORTD is out. We could have used PORTB or PORTC, but I wanted to save those for support efficient bit-based processing of the IR LED since that is timing sensitive.  Since the look-up only happens a few 100 times per second, not that big a deal, right?

 

### Booster circuit

The blue LED sometimes need a larger forward voltage than the battery can directly supply, depending on how new the battery is and the type of blue LED.

The booster uses a modified charge pump to increase the voltage across the blue LED by driving the cathode below zero volts.

More info on the modified charge pump here...

https://wp.josh.com/2017/03/20/blinking-blue-powering-a-3-7-volt-led-from-a-2-4-volt-coin-cell-with-a-0-02-charge-pump/

Q: Why do we need the IO pin?

A: If the battery voltage is high and the blue forward voltage is low, we risk the blue LED coming on dimly from current that flows though the diode. The IO pin goes high to block this current.  

Q: Why do we need the diode? Can't we just connect the capacitor to an IO pin and only activate the pin when we are pushing charger down?

A: Because the IO pins on the AVR have clamping diodes, the second the negative voltage dropped below ~0.2V, the clamping diode would start supplying current. The diode blocks this.  

Q: Can we use a MOSFET here tied to the input of the capacitor to save an IO pin, a BOM line, and the forward drop across the diode? 

A: No matter which way you try to put the MOSFET, you will end up with the body diode either quenching the negative voltage, or letting the quiescent current flow. You could do it with 2 back-to-back MOSFETs, which make make sense if we need that IO pin back (to, say, support the ATMEAG168 P and PA variants again since they have fewer IOs) .

Q: Why pick the 3 pin BAT54 package rather than just a 2 pin diode?

A: Glad you asked!

1. This lets you substitute the extremely popular BAT54C with no changes (the 2nd diode is unused)
2. There is no way to install the 3 pin package backwards
3. Those little SOD-523F packages are PITA to reflow 

## TODO

* Replace D7 with N-CHAN MOSFET (possibly half of a double, with the P-CHAN used for reverse battery protection?). More efficient, fewer parts. But I just can't figure out how to make it work!

## Future Directions

 
* Add a boost for green too so we can run the battery all the way down to 1.8V. Doesn't seem necessary for now, and don't know if there is really enough energy left down there to be worth it (what if we only get like 10 mins?).

* CAPSENSE somewhere? Maybe on button body? Nice to know that a finger is approaching or a hand just waved above. Free, just design time. 
