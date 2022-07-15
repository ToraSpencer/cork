#pragma once


// SIMPLE USAGE:
/*
	  Only include "mesh.decl.h" where-ever you would normally include a
	  header file.  This will avoid including the implementation code in
	  the current compilation unit.
	  Then, create a seperate cpp file which includes "mesh.h" and
	  explicitly instantiates the template with the desired template
	  parameters.

	  By following this scheme, you can prevent re-compiling the entire
	  template implementation in every usage compilation unit and every
	  time those compilation units are recompiled during development.
*/


// ģ������
#include "mesh.decl.h"


// ģ��ʵ��
#include "mesh.tpp"
#include "mesh.remesh.tpp"
#include "mesh.isct.tpp"
#include "mesh.bool.tpp"




