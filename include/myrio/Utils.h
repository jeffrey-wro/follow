#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include "MyRio.h"
#include "NiFpga.h"
#include "I2C.h"
#include <time.h>

class Utils
{
public:
	static NiFpga_Status setupI2CB(NiFpga_Session* myrio_session, MyRio_I2c* i2c);
	static void waitFor (unsigned int secs);
};

#endif
