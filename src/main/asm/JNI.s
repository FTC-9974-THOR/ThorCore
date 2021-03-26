## Utilities for JNI functions.
## Fun fact: JNI functions don't *have* to be written in C or C++. The JNI just looks through an
## object file for a certain symbol, then tries to invoke it with the underlying system's calling
## convention. As long as the symbol exists and is global, the JNI will happily run it. Even if it's
## written in assembly.

#ifndef _JNI_S
#define _JNI_S

## prepends the ThorCore package qualifier to the supplied argument. used for declaring JNI functions.
#define JNI_EXPAND(NAME) Java_org_ftc9974_thorcore_ ## NAME
## marks a JNI function as a global symbol. akin to JNIEXPORT in C/C++.
#define EXPORT_SYMBOL(NAME)\
    .globl JNI_EXPAND(NAME)
## marks a JNI function as executable code that follows the ARM calling convention. akin to JNICALL in C/C++.
#define MARK_FUNCTION(NAME)\
    .type JNI_EXPAND(NAME), %function

#endif