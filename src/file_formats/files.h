#pragma once

// Files类——网格文件IO类

#include <string>
#include "rawMesh.h"

/*
 *  Files provides a wrapper for different file types and a common
 *  data view for the rest of the program.  This wrapper was introduced
 *  to make it easier to support multiple file types using other people's
 *  file importer/exporter code
 */

namespace Files 
{
struct FileVertex : public MinimalVertexData {};
struct FileTriangle : public MinimalTriangleData {};

using FileMesh = RawMesh<FileVertex, FileTriangle>;

// generic filetype functions, these detect which filetype to use by inspecting the filename
int readTriMesh(std::string filename, FileMesh *mesh);
int writeTriMesh(std::string filename, FileMesh *mesh);

// specific filetype functions
int readIFS(std::string filename, FileMesh *mesh);
int writeIFS(std::string filename, FileMesh *mesh);

int readOFF(std::string filename, FileMesh *mesh);
int writeOFF(std::string filename, FileMesh *mesh);

int readOBJ(std::string filename, FileMesh* mesh);
int writeOBJ(std::string filename, FileMesh* mesh);

}  