#pragma once

#ifdef MSPACEOCTREE_LIB
	#define MSPACEOCTREE_LIB_EXPORT __declspec(dllexport)
#else
	#define MSPACEOCTREE_LIB_EXPORT __declspec(dllimport)
#endif // MSpaceOCTree_LIB
