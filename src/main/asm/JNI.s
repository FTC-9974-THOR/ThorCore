#ifndef _JNI_S
#define _JNI_S

#define JNI_EXPAND(NAME) Java_org_ftc9974_thorcore_ ## NAME
#define EXPORT_SYMBOL(NAME)\
    .globl JNI_EXPAND(NAME)
#define MARK_FUNCTION(NAME)\
    .type JNI_EXPAND(NAME), %function

#endif