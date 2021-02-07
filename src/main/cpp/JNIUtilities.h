//
// Created by user on 2/6/21.
//

#ifndef ThorCore_JNIUtilities_H
#define ThorCore_JNIUtilities_H

#include <jni.h>
#include <string>
#include <fmt/include/fmt/core.h>

jclass tryFindClass(JNIEnv* env, const char* className);

jint throwJNIException(JNIEnv* env, const char* className, const char* message);

jint throwOutOfMemoryError(JNIEnv* env, const char* message);
jint throwOutOfMemoryError(JNIEnv* env, const std::string& message);

jint throwIOException(JNIEnv* env, const char* message);
jint throwIOException(JNIEnv* env, const std::string& message);

jint throwNoSuchElementException(JNIEnv* env, const char* message);
jint throwNoSuchElementException(JNIEnv* env, const std::string& msg);

#endif
