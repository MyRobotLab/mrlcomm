/*
 * String.h
 *
 *  Created on: Nov 3, 2016
 *      Author: gperry
 */

#ifndef VIRTUAL_STRING_H_
#define VIRTUAL_STRING_H_

#define F(literal) literal
class StringSumHelper;

class String {
public:
	char* cstr;

	String(){};
	String(const char *cstr = ""){};
	String(const String &str){};
	const char* c_str() const { return cstr; };
	explicit String(char c){};
	explicit String(unsigned char, unsigned char base=10){};
	explicit String(int, unsigned char base=10){};
	explicit String(unsigned int, unsigned char base=10){};
	explicit String(long, unsigned char base=10){};
	explicit String(unsigned long, unsigned char base=10){};
	explicit String(float, unsigned char decimalPlaces=2){};
	explicit String(double, unsigned char decimalPlaces=2){};
	virtual ~String(){};
	const int length() const {return 0;};
	unsigned char operator[](int){return 0;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, const String &rhs){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, const char *cstr){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, char c){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, unsigned char num){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, int num){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, unsigned int num){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, long num){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, unsigned long num){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, float num){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
	friend StringSumHelper & operator + (const StringSumHelper &lhs, double num){StringSumHelper &a= const_cast<StringSumHelper&>(lhs); return a;};
};

class StringSumHelper : public String
{
public:
	StringSumHelper(const String &s) : String(s) {}
	StringSumHelper(const char *p) : String(p) {}
	StringSumHelper(char c) : String(c) {}
	StringSumHelper(unsigned char num) : String(num) {}
	StringSumHelper(int num) : String(num) {}
	StringSumHelper(unsigned int num) : String(num) {}
	StringSumHelper(long num) : String(num) {}
	StringSumHelper(unsigned long num) : String(num) {}
	StringSumHelper(float num) : String(num) {}
	StringSumHelper(double num) : String(num) {}
};
#endif /* VIRTUAL_SERIAL_H_ */
