#ifndef __DFRobot_SIMSMS_H__
#define __DFRobot_SIMSMS_H__

#include "DFRobot_SIMcore.h"

class DFRobot_SIMsms : public DFRobot_SIMcore
{
public:
    bool             beginSMS(const char* to);
    void             editSMS(const char* c);
    bool             sendSMS(void);
};
#endif