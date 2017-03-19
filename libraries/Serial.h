/*
 * Serial.h
 *
 *  Created on: 26 févr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_SERIAL_H_
#define LIBRARIES_SERIAL_H_


class Serial {
public:
	Serial();
	Serial(char *);
	void print(const char *string_);
	void print(float number);
	void print(int number);
	void print(uint32_t number);

    void println(const char *string_);
	void println(float number);
	void println(int number);
	void println(uint32_t number);
	void println(void);

private:
    char *name;
};


#endif /* LIBRARIES_SERIAL_H_ */
