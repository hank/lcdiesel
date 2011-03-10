/*  Defines custom characters.  
    Copyright 2011 Erik Gregg. All rights reserved.

    Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY ERIK GREGG ``AS IS'' AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL ERIK GREGG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of Erik Gregg.
*/
/* To use, just #define CHAR_USE_x where x is the
   name of the desired character in all caps.  Then access it in your program
   with _char_x where x is the lowercase name.
   
   The reason these are not enabled by default is to keep program size down.
   For instance, the ATtiny2313 only has 2K of flash :)
*/
#ifndef CHARS_H
#define CHARS_H

#ifdef CHAR_USE_HEART
static const PROGMEM unsigned char _char_heart[] = 
    {0x00, 0x1B, 0x1F, 0x1F, 0x0E, 0x04, 0x00, 0x00};
#endif

#ifdef CHAR_USE_OPEN_RECTANGLE
static const PROGMEM unsigned char _char_open_rectangle[] = 
    {0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F};
#endif

#ifdef CHAR_3X3_TYPE
// http://en.wikipedia.org/wiki/3x3
static const PROGMEM unsigned char _char_3x3_a[] = 
    {0x04, 0x0E, 0x0A};
static const PROGMEM unsigned char _char_3x3_b[] = 
    {0x0C, 0x0E, 0x0E};
static const PROGMEM unsigned char _char_3x3_c[] = 
    {0x0E, 0x08, 0x0E};
static const PROGMEM unsigned char _char_3x3_d[] = 
    {0x0C, 0x0A, 0x0C};
static const PROGMEM unsigned char _char_3x3_e[] = 
    {0x0E, 0x0C, 0x0E};
static const PROGMEM unsigned char _char_3x3_f[] = 
    {0x0E, 0x0C, 0x08};
static const PROGMEM unsigned char _char_3x3_g[] = 
    {0x0C, 0x0A, 0x0E};
static const PROGMEM unsigned char _char_3x3_h[] = 
    {0x0A, 0x0E, 0x0A};
static const PROGMEM unsigned char _char_3x3_i[] = 
    {0x0E, 0x04, 0x0E};
static const PROGMEM unsigned char _char_3x3_j[] = 
    {0x02, 0x0A, 0x0E};
static const PROGMEM unsigned char _char_3x3_k[] = 
    {0x0A, 0x0C, 0x0A};
static const PROGMEM unsigned char _char_3x3_l[] = 
    {0x08, 0x08, 0x0E};
static const PROGMEM unsigned char _char_3x3_m[] = 
    {0x0E, 0x0E, 0x0A};
static const PROGMEM unsigned char _char_3x3_n[] = 
    {0x0E, 0x0A, 0x0A};
static const PROGMEM unsigned char _char_3x3_o[] = 
    {0x0E, 0x0A, 0x0E};
static const PROGMEM unsigned char _char_3x3_p[] = 
    {0x0E, 0x0E, 0x08};
static const PROGMEM unsigned char _char_3x3_q[] = 
    {0x0E, 0x0E, 0x02};
static const PROGMEM unsigned char _char_3x3_r[] = 
    {0x0E, 0x08, 0x08};
static const PROGMEM unsigned char _char_3x3_s[] = 
    {0x06, 0x04, 0x0C};
static const PROGMEM unsigned char _char_3x3_t[] = 
    {0x0E, 0x04, 0x04};
static const PROGMEM unsigned char _char_3x3_u[] = 
    {0x0A, 0x0A, 0x0E};
static const PROGMEM unsigned char _char_3x3_v[] = 
    {0x0A, 0x0A, 0x04};
static const PROGMEM unsigned char _char_3x3_w[] = 
    {0x0A, 0x0E, 0x0E};
static const PROGMEM unsigned char _char_3x3_x[] = 
    {0x0A, 0x04, 0x0A};
static const PROGMEM unsigned char _char_3x3_y[] = 
    {0x0A, 0x04, 0x04};
static const PROGMEM unsigned char _char_3x3_z[] = 
    {0x0C, 0x04, 0x06};
static const PROGMEM unsigned char _char_3x3_1[] = 
    {0x0C, 0x04, 0x0E};
static const PROGMEM unsigned char _char_3x3_2[] = 
    {0x0C, 0x04, 0x06};
static const PROGMEM unsigned char _char_3x3_3[] = 
    {0x0E, 0x06, 0x0E};
static const PROGMEM unsigned char _char_3x3_4[] = 
    {0x0A, 0x0E, 0x02};
static const PROGMEM unsigned char _char_3x3_5[] = 
    {0x06, 0x04, 0x0C};
static const PROGMEM unsigned char _char_3x3_6[] = 
    {0x08, 0x0E, 0x0E};
static const PROGMEM unsigned char _char_3x3_7[] = 
    {0x0E, 0x02, 0x02};
static const PROGMEM unsigned char _char_3x3_8[] = 
    {0x06, 0x0E, 0x0E};
static const PROGMEM unsigned char _char_3x3_9[] = 
    {0x0E, 0x0E, 0x02};
static const PROGMEM unsigned char _char_3x3_space[] = 
    {0x00, 0x00, 0x00};
#endif

#endif // CHARS_H
