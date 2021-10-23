//
// Created by fortraan on 21/10/2021.
//

#include "JNINativeImageByteBuffer.h"

extern "C" jlong Java_org_ftc9974_thorcore_vision_NativeImageByteBuffer_nativeGetPointer(JNIEnv* env, jobject obj, jobject buf) {
    return (jlong) env->GetDirectBufferAddress(buf);
}
