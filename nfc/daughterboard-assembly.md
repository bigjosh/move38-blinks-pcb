# How to make an NFC daughterboard blink

## Check that you have a compatible blink
Some other batches with 328PBs on them have the SP port pins swapped, which does not work at all. 

![image](https://user-images.githubusercontent.com/5520281/163684815-d68ea1fa-eebe-4880-a6ed-42140a4304cb.png)

## Program the blink with the NFC firmware. 
This will also doublecheck that you have a 328PB or else it will not program. You can use any of the sample sketches to do the download, all that matters is that the right BIOS gets in. 

![image](https://user-images.githubusercontent.com/5520281/163684860-3fe8352c-899f-4ff9-a389-7cfc42d80d08.png)

## Assemble the front of the PCB
Note the orientation of the IC with the dot in the upper left corner.

![image](https://user-images.githubusercontent.com/5520281/163684660-a88f42ec-7e73-44f4-9477-de55b55e9f5d.png)

## Wrap the coil
I used 14 turns of 30 AWG and that worked with my android and old iPhone, but you should test with lots of iPhones and find a count that works reliabily with all. 
A couple of drops of Crazy Glue helps hold the wire in place after you wind. 

![image](https://user-images.githubusercontent.com/5520281/163684678-61bbd5ce-4b8f-44c7-a307-afbde9047b14.png)

## Assemble back of board with coil
Scraping the enamel off the wires makes soldering easier, but you can also solder directly and it should burn off. 

![image](https://user-images.githubusercontent.com/5520281/163685715-510aab77-423f-46ec-b3b8-3509918a16b2.png)

## Stick a 4 pin JST into the back of the blink
Then put on the cover to hold it in place. You can also try a header, but I think it will be harder to hold in place. 

![image](https://user-images.githubusercontent.com/5520281/163684941-9fd7e04a-f1f9-46b8-9c98-ea2d126c8dcb.png)

## Solder the PCB to the pins of the JST that are stocking out on the front side of the blink
Make sure the solder wets both the NFC board pads and the SP connector pads on the blink PCB.

![image](https://user-images.githubusercontent.com/5520281/163684989-2774a28a-a67b-478e-bc34-e09fb7f84403.png)

This is the tricky part. This is a setup that worked for me.

![image](https://user-images.githubusercontent.com/5520281/163684977-13f6aadd-3eea-471d-9e86-d53dc669e411.png)

## Carefully put the coil on the top of the blink
Don't break the wires. 

## Do initial setup of the ST25DV NFC chip 
You need to [set two registers](https://github.com/bigjosh/Move38-BlinkBIOS/blob/add-nfc/NFC.md#setup).

In Android ST app, goto `Set registers` and set the ones in red here and press the save button. If it asks for a password, the password is all `0x00`'s. 

![image](https://user-images.githubusercontent.com/5520281/163686625-3c6e5a34-bb6f-41f4-be3e-7e1494325368.png)

Note that I can not figure out how to do this on the iPhone version of the ST app.

We will want to add the ability to do this to the sample app I made or the real app (maybe under a hidden "initialize blink" option). The new lines will look like...

```

            
 /* Present PWD MB */
  iso15693Tag.customCommand(requestFlags: RequestFlag(rawValue: 0x02), customCommandCode: 0xB3, customRequestParameters: Data(bytes: [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])){ (response: Data, error: Error?) in
      if error != nil {
          session.invalidate(errorMessage: "Present PWD. Please try again."+error!.localizedDescription)
          print(error!.localizedDescription)
          return
      }
  }

  /* Set static registers */
  // Command code 0xa1="Write Configuration", request paramaters are the register number &  the new value

  /* Set GPO config register */
  iso15693Tag.customCommand(requestFlags: RequestFlag(rawValue: 0x02), customCommandCode: 0xa1, customRequestParameters: Data(_: [0x00,0x80]))  { (response: Data, error: Error?) in
      print("in set GPO reg send callback")
      guard error == nil else {
          session.invalidate(errorMessage: "Could not send GPO set register command:"+error!.localizedDescription)
          return
      }

      // The blink controls this number. TODO: Actually figure it out, probably *much* shorter than this
      DispatchQueue.main.asyncAfter(deadline: .now() + .milliseconds(100), execute: {

      /* Set mailbox config register */
      iso15693Tag.customCommand(requestFlags: RequestFlag(rawValue: 0x02), customCommandCode: 0xa1, customRequestParameters: Data(_: [0x0d,0x01]))   { (response: Data, error: Error?) in
          print("in set MB reg send callback")
          guard error == nil else {
              session.invalidate(errorMessage: "Could not send MB set register command:"+error!.localizedDescription)
              return
          }

      ...
```

You might also someday want to set the NDEF records for the tile here to either point to a help page or open the app or whatever you want.
