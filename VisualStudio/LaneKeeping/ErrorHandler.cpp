#include "ErrorHandler.h"

void ErrorHandler::notice(const ErrorCode& code) {
	std::cout << "Error [" << code << "] has occured!\n";
}

void ErrorHandler::critical(const ErrorCode& code){
	std::cout << "Error [" << code << "] has occured!\n";
}
