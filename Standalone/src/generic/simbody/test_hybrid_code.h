#ifndef _TEST_HYBRID_H_
#define _TEST_HYBRID_H_

//inspired by http://research.engineering.wustl.edu/~beardj/Mixed_C_C++.html


//function used by C++ code only

#ifdef __cplusplus

	void print_cpp(void);

#endif

//function used by C and C++ code (must used only C compatible input/output types)

#ifdef __cplusplus
extern "C" {
#endif
	int get_force(void);
#ifdef __cplusplus
}
#endif

#endif