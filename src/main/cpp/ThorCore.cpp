//
// Created by user on 2/6/21.
//

#include <android/log.h>
#include <jni.h>

extern "C" JNIEXPORT void JNICALL Java_org_ftc9974_thorcore_NativeCodeLoader_logLoaded(
        JNIEnv* env, jclass cls) {
    __android_log_print(ANDROID_LOG_INFO, "ThorCoreNative", "loaded modules: libusb, libpixyusb2, NEONVision");
}