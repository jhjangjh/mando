#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <filesystem>
#include <thread>
#include <sstream>
#include <regex>

class transForm
{

public:

	static std::vector<std::string> split(std::string str, char Delimiter);
	static double toNumber(std::string s);

};



std::vector<std::string> transForm::split(std::string str, char Delimiter) 
{
	std::istringstream iss(str);             
	std::string buffer;                      
	std::vector<std::string> result;

	while (getline(iss, buffer, Delimiter)) 
	{
		result.push_back(buffer);               
	}

	return result;
}

double transForm::toNumber(std::string s) 
{
	std::istringstream ss(s);
	double x;
	ss >> x;
	return x;
}