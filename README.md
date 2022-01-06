# InterfaceDriver

It is a code from my organization - NPO GIPO.

This PCB is responsible for the start of the whole device:
- turns on/off a system power
- heats the device
- communicates with another PCBs in the device

There are two projects:
- an old project, which was developed by my elder colleagues
- after I refactored

I cannot blame old unreadable code because it was developed only a few years and it worked.
But I am proud of that I coped with refactoring and made it readable (and still working).

About refactoring.

1) Bad naming (not sustained registers, grammatical mistakes, one-letter variables)
2) Everything is allocated in main.c (cannot find needed function)
3) No abstracts, no structured structures, too few comments




