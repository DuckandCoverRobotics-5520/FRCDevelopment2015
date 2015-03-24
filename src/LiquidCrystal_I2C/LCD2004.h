/*
 * Copyright (c) 2015 Westwood Robotics <code.westwoodrobotics@gmail.com>.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 */




/**
 * LCD CLass for LCM2004
 *
 * @see http://www.wvshare.com/datasheet/LCD_en_PDF/HD44780.pdf
 *
 * @author Austin Reuland <amreuland@gmail.com>
*/
class LCD2004_I2C: public I2C {
protected:
   static const uint8_t  I2C_ADDR = 0x20;
//I2C register
   static const uint8_t  LCD_CLEARDISPLAY = 0x01;
   static const uint8_t  LCD_RETURNHOME = 0x02;
   static const uint8_t  LCD_ENTRYMODESET = 0x04;
   static const uint8_t  LCD_DISPLAYCONTROL = 0x08;
   static const uint8_t  LCD_CURSORSHIFT = 0x10;
   static const uint8_t  LCD_FUNCTIONSET = 0x20;
   static const uint8_t  LCD_SET_CGRAM_ADDR = 0x40;
   static const uint8_t  LCD_SET_DDRAM_ADDR = (byte) 0x80;

   static const uint8_t  LCD_ENTRY_RIGHT = 0x00;
   static const uint8_t  LCD_ENTRY_LEFT = 0x02;
   static const uint8_t  LCD_ENTRY_SHIFT_INCREMENT = 0x01;
   static const uint8_t  LCD_ENTRY_SHIFT_DECREMENT = 0x00;

   static const uint8_t  LCD_DISPLAY_ON = 0x04;
   static const uint8_t  LCD_DISPLAY_OFF = 0x00;
   static const uint8_t  LCD_CURSOR_ON = 0x02;
   static const uint8_t  LCD_CURSOR_OFF = 0x00;
   static const uint8_t  LCD_BLINK_ON = 0x01;
   static const uint8_t  LCD_BLINK_OFF = 0x00;

   static const uint8_t  LCD_DISPLAYMOVE = 0x08;
   static const uint8_t  LCD_CURSORMOVE = 0x00;
   static const uint8_t  LCD_MOVE_RIGHT = 0x04;
   static const uint8_t  LCD_MOVE_LEFT = 0x00;
   static const uint8_t  COMMAND = 0x00;
public:
	explicit LCD2004_I2C(Port port);
	virtual ~LCD2004_I2C();

   virtual void DispData()
   {
   /*
	1. Send a start sequence
	2. Send the I2C address of the slave with the R/W bit low (even address)
	3. Send the internal register number you want to write to
	4. Send the data byte
	5. [Optionally, send any further data bytes]
	6. Send the stop sequence.
*/
   }
};
