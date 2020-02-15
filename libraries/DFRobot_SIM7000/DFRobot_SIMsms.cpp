#include "DFRobot_SIMsms.h"

bool   DFRobot_SIMsms::beginSMS(const char* to)
{
    if(check_send_cmd("AT+CMGF=1\r\n","OK")){
        delay(100);
    }else{
        closeCommand();
        return false;
    }
    send_cmd("AT+CMGS=\"");
    send_cmd(to);
    if(check_send_cmd("\"\r\n",">")){
        delay(100);
        setCommandCounter(2);
        return true;
    }else{
        closeCommand();
        return false;
    }
}

void   DFRobot_SIMsms::editSMS(const char* c)
{
    if(getCommandCounter() == 2){
        send_cmd(c);
        setCommandCounter(3);
    }else{
        closeCommand();
    }
}

bool   DFRobot_SIMsms::sendSMS(void)
{
    if(getCommandCounter() == 3){
        if(check_send_cmd("","+CMGS")){
            closeCommand();
            return true;
        }else{
            closeCommand();
            return false;
        }
    }else{
        closeCommand();
        return false;
    }
}
