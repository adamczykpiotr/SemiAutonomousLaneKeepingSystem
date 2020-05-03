#pragma once
#include <chrono>
#include <string_view>
#include <iostream>

class Timer {
	std::string_view m_title;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_startPoint;
	long long m_scaler;

public:
	Timer(std::string_view title, long long scaler = 1) {
		m_startPoint = std::chrono::high_resolution_clock::now();
		m_title = title;
		m_scaler = scaler;
	}

	~Timer() {
		std::chrono::time_point<std::chrono::high_resolution_clock> endPoint = std::chrono::high_resolution_clock::now();
		auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_startPoint).time_since_epoch().count();
		auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endPoint).time_since_epoch().count();

		auto duration = end - start;
		if (m_scaler != 1) {
			std::cout << "On average ";
			duration /= m_scaler;
		}
		auto ms = duration * 0.001;

		std::cout << m_title << " took " << duration << " us (" << ms << " ms)\n";
	}
};

