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


int readOFF(string filename, FileMesh *data)
{
    if(!data) 
        return 1;
    
    ifstream in;                                            // will close on exit from this read function
    in.open(filename.c_str());
    if(!in) 
        return 1;
    
    // "OFF"
    string filetype;
    in >> filetype;
    if(filetype != "OFF") 
        return 1;
    
    // counts of things
    int numvertices, numfaces, numedges;
    in >> numvertices >> numfaces >> numedges;
    if(!in) 
        return 1;
    data->vertices.resize(numvertices);
    data->triangles.resize(numfaces);
    
    // vertex data
    for(auto &v : data->vertices) 
    {
        Vec3d &p = v.pos;
        in >> p.x >> p.y >> p.z;
    }
    if(!in) 
        return 1;
    
    // face data
    for(auto &tri : data->triangles) 
    {
        int polysize;
        in >> polysize;
        if(polysize != 3)   
            return 1;
        
        in >> tri.a >> tri.b >> tri.c;
    }

    if(!in) 
        return 1;
    
    return 0;
}


int writeOFF(string filename, FileMesh *data)
{
    if(!data) 
        return 1;
    
    ofstream out;
    out.open(filename.c_str());
    if(!out) 
        return 1;
    
    out << "OFF" << endl;

    int numvertices = data->vertices.size();
    int numfaces = data->triangles.size();
    out << numvertices << ' ' << numfaces << ' ' << 0 << endl;
    
    // 写顶点数据
    for(const auto &v : data->vertices) 
    {
        const Vec3d &p = v.pos;
        out << p.x << ' ' << p.y << ' ' << p.z << endl;
    }
    
    // 写三角片数据
    for(const auto &tri : data->triangles) 
        out << "3 " << tri.a << ' ' << tri.b << ' ' << tri.c << endl;

    if(!out) 
        return 1;
    
    return 0;
}

} 
