FATFS
=====
The FATFS provides file system for the storage medium
Based on the storage size the file system will be organized into FAT12/FAT16/FAT32
The FATFS application interface provides File/Directory  Access, File/Directory Management and Volume Management


Features
--------

* Very small footprint for FATFS API  implementation
* Windows compatible FAT file system
* File system settings are configurable

Dependencies
------------

::

            FATFS
              /\
             /  \
            /    \
         DISKIO  RTT
           /\
          /  \
         /    \
     SD/MMC   USB MSC
       /        \ 
      /          \
    MCI          USB

* Generic disk access layer to perform DiskIO functions
* RTT calendar interface
* SD/MMC media accessor
* USB mass storage device media accessor
* Peripheral that supports multimedia card interface (MCI)
* USB Driver


Example Applications
--------------------

* Generic FATFS application to demonstrate the API access
* POSIX standard file system API access

Limitations
-----------

* FAT sub-types: FAT12, FAT16 and FAT32.
* Number of open files: Unlimited. (depends on available memory)
* Number of volumes: Upto 10.
* File size: Upto 4G-1 bytes. (by FAT specs.)
* Volume size: Upto 2T bytes at 512 bytes/sector. (by FAT specs.)
* Cluster size: Upto 64K bytes at 512 bytes/sector. (by FAT specs.)
* Sector size: 512, 1024, 2048 and 4096 bytes. (by FAT specs.)

Reference
---------

* :API Documentation: '<../fatfs/doc/00index_e.html>'
* http://elm-chan.org/fsw/ff/00index_e.html 

License
--------

::

/*----------------------------------------------------------------------------/
/  FatFs - FAT file system module  R0.11                 (C)ChaN, 2015
/-----------------------------------------------------------------------------/
/ FatFs module is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/

