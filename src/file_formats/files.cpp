#include "files.h"
#include <iostream>

using std::cout;
using std::cerr;
using std::endl;


namespace Files 
{
using std::string;


int readTriMesh(string filename, FileMesh *mesh)
{
    int lastdot = filename.find_last_of('.');
    if(lastdot < 0) return 1;
    string suffix = filename.substr(lastdot, filename.length()-lastdot);
    if (suffix == ".ifs")
        return readIFS(filename, mesh);
    else if (suffix == ".off")
        return readOFF(filename, mesh);
    else if (suffix == ".obj")
        return readOBJ(filename, mesh);
    else
        return 1;
}


int writeTriMesh(string filename, FileMesh *mesh)
{
    int lastdot = filename.find_last_of('.');
    if(lastdot < 0) 
        return 1;
    string suffix = filename.substr(lastdot, filename.length()-lastdot);
    if(suffix == ".ifs")
        return writeIFS(filename, mesh);
    else if (suffix == ".off")
        return writeOFF(filename, mesh);
    else if (suffix == ".obj")
        return writeOBJ(filename, mesh);
    else
        return 1;
}

}  
