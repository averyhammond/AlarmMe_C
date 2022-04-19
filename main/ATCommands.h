#ifndef SIM800l_CMDSH
#define SIM800l_CMDSH

#include <stdint.h>

#define GSM_OK_Str "OK"


typedef struct
{
    char        *cmd;
    uint16_t    cmdSize;
    uint16_t    timeoutMs;
    uint16_t    delayMs;
    uint8_t        skip;
}GSM_Cmd;

static GSM_Cmd cmd_AT =
{
    .cmd = "AT\r\n",
    .cmdSize = sizeof("AT\r\n")-1,
    .timeoutMs = 3000,
    .delayMs = 0,
    .skip = 0,
};

/*static GSM_Cmd cmd_signalQuality = 
{
    .cmd = "AT+CSQ\r\n",
    .cmdSize = sizeof("AT+CSQ\r\n")-1,
    .cmdResponseOnOk = GSM_OK_Str,
    .timeoutMs = 30000,
    .delayMs = 2000,
    .skip = 0,
};

static GSM_Cmd cmd_SMSMode = 
{
    .cmd = "AT+CMGF=1\r\n",
    .cmdSize = sizeof("AT+CMGF=1\r\n")-1,
    .cmdResponseOnOk = GSM_OK_Str,
    .timeoutMs = 30000,
    .delayMs = 2000,
    .skip = 0,
};

static GSM_Cmd cmd_sendSMS = 
{
    .cmd = "AT+CMGS="+16092357269"\r\n",
    .cmdSize = sizeof("AT+CMGS="+16092357269"\r\n")-1,
    .cmdResponseOnOk = GSM_OK_Str,
    .timeoutMs = 30000,
    .delayMs = 2000,
    .skip = 0,
};

static GSM_Cmd cmd_message = 
{
    .cmd = "This is SIM800L\032",
    .cmdSize = sizeof("This is SIM800L\032")-1,
    .cmdResponseOnOk = GSM_OK_Str,
    .timeoutMs = 30000,
    .delayMs = 2000,
    .skip = 0,
};

static GSM_Cmd cmd_error = 
{
    .cmd = "AT+CMEE=1\r\n",
    .cmdSize = sizeof("AT+CMEE=1\r\n")-1,
    .cmdResponseOnOk = GSM_OK_Str,
    .timeoutMs = 30000,
    .delayMs = 2000,
    .skip = 0,
};

static GSM_Cmd cmd_GSMMode = 
{
    .cmd = "AT+CSCS=\"GSM\"\r\n",
    .cmdSize = sizeof("AT+CSCS=\"GSM\"\r\n")-1,
    .cmdResponseOnOk = GSM_OK_Str,
    .timeoutMs = 30000,
    .delayMs = 2000,
    .skip = 0,
};

static GSM_Cmd cmd_checkCSCS = 
{
    .cmd = "AT+CSCS?\r\n",
    .cmdSize = sizeof("AT+CSCS?\r\n")-1,
    .cmdResponseOnOk = GSM_OK_Str,
    .timeoutMs = 30000,
    .delayMs = 2000,
    .skip = 0,
};*/

#endif