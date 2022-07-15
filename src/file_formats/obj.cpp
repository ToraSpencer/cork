#include "files.h"

#include <iostream>
#include <fstream>
using std::ifstream;
using std::ofstream;
using std::endl;


namespace Files 
{

using std::string;
using std::vector;


unsigned ReadNextValidData(char*& pszBuf, unsigned& nCount, char* validData, const unsigned nMaxSize)
{
    unsigned nIndx = 0;

    while ((pszBuf[0] == ' ') ||
        (pszBuf[0] == '\n') ||
        (pszBuf[0] == '\t') ||
        (pszBuf[0] == '\r'))
    {
        pszBuf++;
        nCount++;
    }

    while ((pszBuf[0] != ' ') &&
        (pszBuf[0] != '\n') &&
        (pszBuf[0] != '\t') &&
        (pszBuf[0] != '\r') &&
        (pszBuf[0] != '\null') &&
        (pszBuf[0] != 0) &&
        (nIndx < nMaxSize))
    {
        validData[nIndx++] = pszBuf[0];
        pszBuf++;
        nCount++;
    }
    validData[nIndx] = 0;
    return nIndx;
}


int readOBJ(string filename, FileMesh *data)
{
	char* pTmp = nullptr;
	std::ifstream ifs(filename.c_str()); 
	if (false == ifs.is_open())
		return 1;
 
	std::streampos   pos = ifs.tellg();      
	ifs.seekg(0, std::ios::end);
	unsigned fileLen = (unsigned)ifs.tellg();
	if (0 == fileLen)
		return 1;

	ifs.seekg(pos);     
	char* pFileBuf = new char[fileLen + 1];
	std::memset(pFileBuf, 0, fileLen + 1);
	ifs.read(pFileBuf, fileLen);
	char tmpBuffer[1024];
	unsigned nMaxSize = 1024;
	pTmp = pFileBuf;
	unsigned nReadLen = 0;
	unsigned nRet = 0;

	while (nReadLen < fileLen)
	{
		nRet = ReadNextValidData(pTmp, nReadLen, tmpBuffer, nMaxSize);
		if (0 == nRet)
			break;

		// ¶¥µãÐÅÏ¢		
		if (std::strcmp(tmpBuffer, "v") == 0)
		{
			FileVertex vert;
			nRet = ReadNextValidData(pTmp, nReadLen, tmpBuffer, nMaxSize);
			if (0 == nRet)
				break;
			vert.pos.x = (float)atof(tmpBuffer);
			nRet = ReadNextValidData(pTmp, nReadLen, tmpBuffer, nMaxSize);
			if (0 == nRet)
				break;
			vert.pos.y = (float)atof(tmpBuffer);
			nRet = ReadNextValidData(pTmp, nReadLen, tmpBuffer, nMaxSize);
			if (0 == nRet)
				break;
			vert.pos.z = (float)atof(tmpBuffer);
			data->vertices.push_back(vert);
		}
		else if (std::strcmp(tmpBuffer, "f") == 0)
		{
			FileTriangle surf;
			nRet = ReadNextValidData(pTmp, nReadLen, tmpBuffer, nMaxSize);
			if (0 == nRet)
				break;
			surf.a = atoi(tmpBuffer) - 1;
			nRet = ReadNextValidData(pTmp, nReadLen, tmpBuffer, nMaxSize);
			if (0 == nRet)
				break;
			surf.b = atoi(tmpBuffer) - 1;

			nRet = ReadNextValidData(pTmp, nReadLen, tmpBuffer, nMaxSize);
			if (0 == nRet)
				break;
			surf.c = atoi(tmpBuffer) - 1;
			data->triangles.push_back(surf);
		}
	}
	delete[] pFileBuf;
    
    return 0;
}


int writeOBJ(string filename, FileMesh* data) 
{
	std::ofstream dstFile(filename.c_str());

	unsigned nSize = data->vertices.size();
	for (unsigned j = 0; j < nSize; j++)
	{
		char szBuf[256] = { 0 };
		auto vert = data->vertices[j];
		sprintf_s(szBuf, 256, "v %f %f %f", vert.pos.x, vert.pos.y, vert.pos.z);
		dstFile << szBuf << "\n";
	}

	nSize = data->triangles.size();
	for (unsigned j = 0; j < nSize; ++j)
	{
		char szBuf[256] = { 0 };
		const auto& tri = data->triangles[j];
		sprintf_s(szBuf, 256, "f %d %d %d", tri.a + 1, tri.b + 1, tri.c + 1);
		dstFile << szBuf << "\n";
	}

	return 0;
}

} 
