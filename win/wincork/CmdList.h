#pragma once
#include "cork.h"
#include <iostream>
#include <sstream>


// 控制台指令管理器；
class CmdList 
{
public:
    CmdList();
    ~CmdList() {}

    void regCmd(string name, string helptxt, std::function< void(std::vector<string>::iterator&, const std::vector<string>::iterator&) > body);

    void printHelp(ostream& out);

    void runCommands(std::vector<string>::iterator& arg_it,
        const std::vector<string>::iterator& end_it);

private:
    struct Command 
    {
        string name;                // e.g. "show" will be invoked with option "-show"
        string helptxt;             // lines to be displayed
        std::function< void(std::vector<string>::iterator&,
            const std::vector<string>::iterator&) > body;
    };

    std::vector<Command> commands;
};
