#include "Profile.h"
#include <iostream>

Profile::Profile(const std::string& name) : name(name), start(std::chrono::high_resolution_clock::now()) {}

Profile::~Profile() {
	auto duration = std::chrono::high_resolution_clock::now() - start;
	auto s = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
	auto hours = std::chrono::duration_cast<std::chrono::hours>(duration).count();
	std::cout << "Profile \"" << name << "\": " << s << " s" << std::endl;
	std::cout << "Profile \"" << name << "\": " << ms << " ms" << std::endl;
	std::cout << "Profile \"" << name << "\": " << hours << " hour" << std::endl;
}
