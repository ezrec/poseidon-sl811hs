Poseidon support for SL811HS based devices

******************************************************
**            !!!!! DEADLY WARNING !!!!!            **
******************************************************
* THIS DRIVER IS PRE-RELEASE, AND WILL CORRUPT DATA! *
******************************************************

This driver suppors the Pathway (Clockport) and Thylacine (Zorro) boards.

Use thylacine.device for Thylacine boards. Device unit numbers correspond
to the Zorro board order (ie Unit 0 for the first detected Thylacine,
Unit 1 for the next, etc).

Use pathway.device for Pathway clockport USB adapters.
Unit numbers are as follows:

Unit    Base      Comment
  0    0xd80001   A1200 clockport
  1    0xd84001   Zorro IV
  2    0xd88001   Zorro IV
  3    0xd8c001   Zorro IV
  4    0xd90001   A604 2nd port

