#pragma once
#ifdef D4PCR_DLL_EXPORT   // 如果在编译DLL时定义了这个宏
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllimport)
#endif