#ifndef HD44780_HPP
#define HD44780_HPP

class HD44780
{
public:
	HD44780();

public:
	~HD44780();

public:
	bool init();

	void clear();

	void setCursorPos(int _line, int _column);

	void writeChar(char _c);

	void writeString(const char *_c);

private:
	bool initialized;
};

#endif
