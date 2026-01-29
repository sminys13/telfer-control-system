#pragma once
#include "config.h"

void setError(ErrorType error, const char *message);
void clearError(void);
bool hasError(void);
const char *getErrorMessage(ErrorType error);
