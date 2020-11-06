#pragma once
#include <iostream>

class ErrorHandler {
public:
	enum ErrorCode {
		VideoCaptureError,
	};

	static void notice(const ErrorCode& code);
	static void critical(const ErrorCode& code);
};

