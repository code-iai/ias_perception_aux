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


#include "File.h"

#ifndef WIN32
#ifndef sprintf_s
#define sprintf_s(a,b,c,d) sprintf(a,c,d)
#include <string.h>
#include <stdio.h>
#endif
#endif



linfile::File::~File(void)
{
	delete nextElem;
	nextElem = NULL;
	m_file.close();
}

linfile::File::File(std::string filename, bool bOut, bool bAppend)
{
	if(!bAppend)
	{
		m_file.open(filename.c_str(), bOut ? std::ios_base::out : std::ios_base::in);
	}
	else
	{
		m_file.open(filename.c_str(), std::ios_base::app);
	}
	if(m_file.bad())
		printf("Error opening the file");
	nextElem = new char[LIN__BUFFERSIZE];
}

double linfile::File::GetNextDouble()
{
	m_file.get(nextElem, LIN__BUFFERSIZE, ' ');
	while(nextElem[0] == '#')
	{
		m_file.getline(nextElem, LIN__BUFFERSIZE);
		m_file.get(nextElem, LIN__BUFFERSIZE, ' ');
	}
	double dRet = atof(nextElem);
	m_file.get(nextElem, 2);
	return dRet;
}
int linfile::File::GetNextInt()
{
	m_file.get(nextElem, LIN__BUFFERSIZE, ' ');
	while(nextElem[0] == '#')
	{
		m_file.getline(nextElem, LIN__BUFFERSIZE);
		m_file.get(nextElem, LIN__BUFFERSIZE, ' ');
	}
	int nRet = atoi(nextElem);
	m_file.get(nextElem, 2);
	return nRet;
}
char* linfile::File::GetNextCommentLine()
{
	m_file.getline(nextElem, LIN__BUFFERSIZE);
	return nextElem;
}

char* linfile::File::GetNextWord()
{
	m_file.get(nextElem, LIN__BUFFERSIZE, ' ');
	return nextElem;
}

std::vector<int> linfile::File::GetNextIntVector()
{
	std::vector<int> viReturn;
	int nSize = GetNextInt();
	for(int i = 0; i < nSize; i++)
		viReturn.push_back(GetNextInt());
	return viReturn;
}

std::vector<double> linfile::File::GetNextDoubleVector()
{
	std::vector<double> vdReturn;
	int nSize = GetNextInt();
	for(int i = 0; i < nSize; i++)
		vdReturn.push_back(GetNextDouble());
	return vdReturn;
}

void linfile::File::WriteInt(int nValue)
{
	sprintf_s(nextElem, LIN__BUFFERSIZE, "%d ", nValue);
	int nLen = strlen(nextElem);
	m_file.write(nextElem, nLen);
}

void linfile::File::WriteSingleInt(int nValue)
{
	sprintf_s(nextElem, LIN__BUFFERSIZE, "%d", nValue);
	int nLen = strlen(nextElem);
	m_file.write(nextElem, nLen);
}

void linfile::File::WriteIntAsHex(int nValue)
{
	sprintf_s(nextElem, LIN__BUFFERSIZE, "%X ", nValue);
	int nLen = strlen(nextElem);
	m_file.write(nextElem, nLen);
}

void linfile::File::WriteSingleIntAsHex(int nValue)
{
	sprintf_s(nextElem, LIN__BUFFERSIZE, "%X", nValue);
	int nLen = strlen(nextElem);
	m_file.write(nextElem, nLen);
}

void linfile::File::WriteDouble(const double dValue)
{
	sprintf_s(nextElem, LIN__BUFFERSIZE, "%f ", dValue);
	int nLen = strlen(nextElem);
	m_file.write(nextElem, nLen);
}

void linfile::File::WriteSingleDouble(const double dValue)
{
	sprintf_s(nextElem, LIN__BUFFERSIZE, "%f", dValue);
	int nLen = strlen(nextElem);
	m_file.write(nextElem, nLen);
}

void linfile::File::WriteDoubleLimLength(const double dValue, const int nDigits)
{
	sprintf_s(nextElem, LIN__BUFFERSIZE, "%.1f", dValue);
	int nLen = strlen(nextElem);
	m_file.write(nextElem, nLen);
}

void linfile::File::WriteLine(const char* stLine)
{
	int nLen = strlen(stLine);
	m_file.write(stLine, nLen);
}

void linfile::File::WriteVector(std::vector<int> viData)
{
	std::vector<int>::const_iterator it = viData.begin();
	WriteInt(viData.size());
	while(it != viData.end())
	{
		WriteInt((*it));
		it++;
	}
}

void linfile::File::WriteVector(std::vector<double> vdData)
{
	std::vector<double>::const_iterator it = vdData.begin();
	WriteInt(vdData.size());
	while(it != vdData.end())
	{
		WriteDouble((*it));
		it++;
	}
}
