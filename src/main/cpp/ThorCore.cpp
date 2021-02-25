//
// Created by user on 2/6/21.
//

#include <android/log.h>
#include <jni.h>

#ifndef THORCORE_LOADED_MODULES
#define THORCORE_LOADED_MODULES "ThorCore"
#else
#define THORCORE_LOADED_MODULES "ThorCore " + THORCORE_LOADED_MODULES
#endif

extern "C" JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_NativeCodeLoader_logLoaded(
        JNIEnv* env, jclass cls) {
    __android_log_print(ANDROID_LOG_INFO, "ThorCoreNative", "loaded modules: %s", THORCORE_LOADED_MODULES);
}