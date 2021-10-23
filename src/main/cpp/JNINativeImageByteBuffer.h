//
// Created by fortraan on 21/10/2021.
//

#ifndef ThorCore_JNINativeImageByteBuffer_H
#define ThorCore_JNINativeImageByteBuffer_H

#include <jni.h>
#include "JNIUtilities.h"

extern "C" JNIEXPORT jlong JNICALL Java_org_ftc9974_thorcore_vision_NativeImageByteBuffer_nativeGetPointer(JNIEnv *env, jobject obj, jobject buf);

#endif //ThorCore_JNINativeImageByteBuffer_H
