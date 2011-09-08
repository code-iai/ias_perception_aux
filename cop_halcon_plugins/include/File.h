/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#ifndef FILE_H
#define FILE_H
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h>

namespace linfile
{
#define LIN__BUFFERSIZE 256

class File
{
public:
	File(std::string, bool bOut = true, bool bAppend = false);
	~File(void);
	double GetNextDouble();
	void RestartReading(){m_file.seekg(0);}
	int GetNextInt();
	char* GetNextCommentLine();
	char* GetNextWord();
	std::vector<int> GetNextIntVector();
	std::vector<double> GetNextDoubleVector();
	void WriteInt(const int nValue);
	void WriteSingleInt(int nValue);
	void WriteIntAsHex(int nValue);
	void WriteSingleIntAsHex(int nValue);
	void WriteDouble(const double dValue);
	void WriteSingleDouble(const double dValue);
	void WriteDoubleLimLength(const double dValue, const int nDigits = 1);
	void WriteLine(const char* stLine);
	void WriteVector(std::vector<int> viData);
	void WriteVector(std::vector<double> vdData);
	bool Invalid(){return m_file.bad();}
	void CloseFile(){m_file.close();}
private:
	std::fstream m_file;
	char* nextElem;
};
}
#endif //FILE_H

