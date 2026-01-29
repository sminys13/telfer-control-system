#include "errors.h"
#include <Arduino.h>

// Глобальные структуры у тебя уже есть в main.cpp:
extern SystemData systemData;
extern SystemFlags systemFlags;

void setError(ErrorType error, const char *message)
{
    systemData.activeError = error;

    if (message && message[0])
    {
        strncpy(systemData.errorMessage, message, sizeof(systemData.errorMessage) - 1);
        systemData.errorMessage[sizeof(systemData.errorMessage) - 1] = '\0';
    }
    else
    {
        strncpy(systemData.errorMessage, getErrorMessage(error), sizeof(systemData.errorMessage) - 1);
        systemData.errorMessage[sizeof(systemData.errorMessage) - 1] = '\0';
    }

    systemFlags.isEmergency = (error == ERROR_EMERGENCY_STOP || error == ERROR_LIMIT_SWITCH);
}

void clearError(void)
{
    systemData.activeError = ERROR_NONE;
    systemData.errorMessage[0] = '\0';
    systemFlags.isEmergency = false;
}

bool hasError(void)
{
    return systemData.activeError != ERROR_NONE;
}

const char *getErrorMessage(ErrorType error)
{
    switch (error)
    {
        case ERROR_NONE: return "OK";
        case ERROR_SENSOR_LASER1: return "LASER1 ERR";
        case ERROR_SENSOR_LASER2: return "LASER2 ERR";
        case ERROR_SENSOR_US1: return "US1 ERR";
        case ERROR_SENSOR_US2: return "US2 ERR";
        case ERROR_RS485_COMM: return "RS485 ERR";
        case ERROR_LIMIT_SWITCH: return "LIMIT";
        case ERROR_EMERGENCY_STOP: return "E-STOP";
        case ERROR_MEMORY: return "MEM";
        case ERROR_DISPLAY: return "DISPLAY";
        default: return "ERROR";
    }
}
