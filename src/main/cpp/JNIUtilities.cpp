//
// Created by user on 2/6/21.
//

#include "JNIUtilities.h"

jclass tryFindClass(JNIEnv* env, const char* className) {
    jclass clazz = env->FindClass(className);
    if (clazz == nullptr) {
        if (strcmp(className, "java/lang/NoClassDefFoundError") == 0) {
            // the exception that we would throw in this case is the one that just failed to load,
            // so give up
            throw std::exception();
        }

        throwJNIException(env, "java/lang/NoClassDefFoundError", fmt::format("Could not find class \"{%s}\"", className).c_str());
        // just to be safe, ensure return from this method
        throw std::exception();
    }

    return clazz;
}

jint throwJNIException(JNIEnv* env, const char* className, const char* message) {
    jclass exceptionClass = tryFindClass(env, className);
    return env->ThrowNew(exceptionClass, message);
}

jint throwOutOfMemoryError(JNIEnv* env, const char* message) {
    return throwJNIException(env, "java/lang/OutOfMemoryError", message);
}

jint throwOutOfMemoryError(JNIEnv* env, const std::string& message) {
    return throwOutOfMemoryError(env, message.c_str());
}

jint throwIOException(JNIEnv* env, const char* message) {
    return throwJNIException(env, "java/io/IOException", message);
}

jint throwIOException(JNIEnv* env, const std::string& message) {
    return throwIOException(env, message.c_str());
}

jint throwNoSuchElementException(JNIEnv* env, const char* message) {
    return throwJNIException(env, "java/util/NoSuchElementException", message);
}

jint throwNoSuchElementException(JNIEnv* env, const std::string& message) {
    return throwNoSuchElementException(env, message.c_str());
}

jint throwIllegalAccessError(JNIEnv* env, const char* message) {
    return throwJNIException(env, "java/lang/IllegalAccessError", message);
}

jint throwIllegalAccessError(JNIEnv* env, const std::string& message) {
    return throwIllegalAccessError(env, message.c_str());
}

jint throwIndexOutOfBoundsException(JNIEnv* env, const char* message) {
    return throwJNIException(env, "java/lang/IndexOutOfBoundsException", message);
}

jint throwIndexOutOfBoundsException(JNIEnv* env, const std::string& message) {
    return throwIndexOutOfBoundsException(env, message.c_str());
}