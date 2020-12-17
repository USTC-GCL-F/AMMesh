#pragma once

#include <string>
#include <iostream>

namespace acamcad {
	class Massage
	{
	public:
		Massage() {};
		~Massage() {};

		static void Error(const char* fmt)
		{
			std::cout << "Error: " << fmt << std::endl;
		}
		static void Error(const std::string& fmt)
		{
			std::cout << "Error: " << fmt << std::endl;
		}

		static void Warning(const char* fmt)
		{
			std::cout << "Warning: " << fmt << std::endl;
		}
		static void Warning(const std::string& fmt)
		{
			std::cout << "Warning: " << fmt << std::endl;
		}

		static void Info(const char* fmt)
		{
			std::cout << "Info: " << fmt << std::endl;
		}
		static void Info(const std::string& fmt)
		{
			std::cout << "Info: " << fmt << std::endl;
		}

		static void Out(const char* fmt)
		{
			std::cout << fmt << std::endl;
		}

	};
}