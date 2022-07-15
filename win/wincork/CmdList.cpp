#pragma once
#include "CmdList.h"

CmdList::CmdList()
{
    regCmd("help",
        "-help                  show this help message",
        [this](std::vector<string>::iterator&,  const std::vector<string>::iterator&) 
        {
                printHelp(cout);
                exit(0);
        });
}


void CmdList::regCmd(string name, string helptxt, std::function< void(std::vector<string>::iterator&, \
    const std::vector<string>::iterator&) > body)
{

    Command cmd = {
        name,
        helptxt,
        body
    };

    commands.push_back(cmd);
}


// "cork -help"
void CmdList::printHelp(ostream& out)
{
    out <<
        "Welcome to Cork.  Usage:" << endl <<
        "  > cork [-command arg0 arg1 ... argn]*" << endl <<
        "for example," << endl <<
        "  > cork -union box0.off box1.off result.off" << endl <<
        "Options:" << endl;
    for (auto& cmd : commands)
        out << cmd.helptxt << endl;
    out << endl;
}


void CmdList::runCommands(std::vector<string>::iterator& arg_it,
    const std::vector<string>::iterator& end_it)
{
    while (arg_it != end_it) {
        string arg_cmd = *arg_it;
        if (arg_cmd[0] != '-') {
            cerr << arg_cmd << endl;
            cerr << "All commands must begin with '-'" << endl;
            exit(1);
        }
        arg_cmd = arg_cmd.substr(1);
        arg_it++;

        bool found = true;
        for (auto& cmd : commands) {
            if (arg_cmd == cmd.name) {
                cmd.body(arg_it, end_it);
                found = true;
                break;
            }
        }
        if (!found) {
            cerr << "Command -" + arg_cmd + " is not recognized" << endl;
            exit(1);
        }
    }
}


std::function< void(std::vector<string>::iterator&, const std::vector<string>::iterator&)> genericBinaryOp(\
    std::function< void(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out) >  binop)
{
    return [binop](std::vector<string>::iterator& args, const std::vector<string>::iterator& end)
    {
        // data...
        CorkTriMesh in0;
        CorkTriMesh in1;
        CorkTriMesh out;

        if (args == end) { cerr << "too few args" << endl; exit(1); }
        loadMesh(*args, &in0);
        args++;

        if (args == end) { cerr << "too few args" << endl; exit(1); }
        loadMesh(*args, &in1);
        args++;

        binop(in0, in1, &out);

        if (args == end) { cerr << "too few args" << endl; exit(1); }
        saveMesh(*args, out);
        args++;

        freeCorkTriMesh(&out);

        delete[] in0.vertices;
        delete[] in0.triangles;
        delete[] in1.vertices;
        delete[] in1.triangles;
    };
}


// 在命令行界面运行wincork.exe，"-"之后加入指令；如-union mesh1.off mesh2.off result.off;
/*
        貌似只能读取OFF格式的网格数据；
*/
int runCmd(int argc, char* argv[])

{
    std::cout << "welcome " << std::endl;

    initRand();              // that's useful

    if (argc < 2)
    {
        cout << "Please type 'cork -help' for instructions" << endl;
        exit(0);
    }

    // 存储传给main函数的参数
    std::vector<string> args(argc);         // main函数字符串参数的vector;
    for (int k = 0; k < argc; k++)
        args[k] = argv[k];

    auto arg_it = args.begin();
    arg_it++;

    CmdList cmds;

    // 检测网格是否是watertight网格，即是否完全封闭，且没有自交； -solid
    cmds.regCmd("solid",
        "-solid in              Determine whether the input mesh represents\n"
        "                       a solid object.  (aka. watertight) (technically\n"
        "                         solid == closed and non-self-intersecting)",

        [](std::vector<string>::iterator& args,
            const std::vector<string>::iterator& end)
        {
            CorkTriMesh in;
            if (args == end) { cerr << "too few args" << endl; exit(1); }
            string filename = *args;
            loadMesh(*args, &in);
            args++;

            bool solid = isSolid(in);
            cout << "The mesh " << filename << " is: " << endl;
            cout << "    " << ((solid) ? "SOLID" : "NOT SOLID") << endl;

            delete[] in.vertices;
            delete[] in.triangles;
        });


    // 并运算-union
    cmds.regCmd("union",
        "-union in0 in1 out     Compute the Boolean union of in0 and in1,\n"
        "                       and output the result",
        genericBinaryOp(computeUnion));

    // 减法运算-diff
    cmds.regCmd("diff",
        "-diff in0 in1 out      Compute the Boolean difference of in0 and in1,\n"
        "                       and output the result",
        genericBinaryOp(computeDifference));

    // -isct
    cmds.regCmd("isct",
        "-isct in0 in1 out      Compute the Boolean intersection of in0 and in1,\n"
        "                       and output the result",
        genericBinaryOp(computeIntersection));

    // 异或运算-xor
    cmds.regCmd("xor",
        "-xor in0 in1 out       Compute the Boolean XOR of in0 and in1,\n"
        "                       and output the result\n"
        "                       (aka. the symmetric difference)",
        genericBinaryOp(computeSymmetricDifference));


    cmds.regCmd("resolve",
        "-resolve in0 in1 out   Intersect the two meshes in0 and in1,\n"
        "                       and output the connected mesh with those\n"
        "                       intersections made explicit and connected",
        genericBinaryOp(resolveIntersections));

    cmds.runCommands(arg_it, args.end());

    return 0;
}
