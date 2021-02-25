//
// Created by user on 2/9/21.
//

#include <jni.h>
#include <time.h>
#include <android/log.h>
#include <android/hardware_buffer.h>
#include <android/hardware_buffer_jni.h>
#include <arm_neon.h>

#define TAG "JNINEONVision"

// these link against functions written in assembly, contained in NEONVision.s
//extern "C" long testcall(long a, long b);
//extern "C" long accumThreshold(uint8_t* pixels, uint64_t pixelBufLen, uint64_t* high, uint64_t* low, uint64_t numPixels);
extern "C" JNIEXPORT jlong JNICALL Java_org_ftc9974_thorcore_vision_NEONVision_processYUY2(JNIEnv* env, jobject obj, jlong bufferPtr, jlong bufferLen, jlong highColor, jlong lowColor);

extern "C" {
    JNIEXPORT jboolean JNICALL Java_org_ftc9974_thorcore_vision_NEONVision_canUseNeonAcceleration(
            JNIEnv* env, jclass cls) {
        return USE_NEON_ACCEL;
    }

    /*JNIEXPORT jlong JNICALL Java_org_ftc9974_thorcore_vision_NEONVision_test(
            JNIEnv* env, jobject obj, jlong a, jlong b) {
        auto* testData = new uint32_t[16];
        for (int i = 0; i < 16; i++) {
            testData[i] = 0xff112233;
        }
        uint64_t high = 0x44ffffff;
        uint64_t low = 0x00000000;
        time_t start, end;
        time(&start);
        long sum = accumThreshold(reinterpret_cast<uint8_t*>(testData), 64, &high, &low, 16);
        time(&end);
        __android_log_print(ANDROID_LOG_INFO, TAG, "test sum: %ld 0x%lx", sum, sum);
        delete[] testData;
        // java stores numbers in MSB, so if it isn't converted properly to the native endianness
        // this should return 2^63 unsigned. java should interpret that as -2^63. so if this returns
        // a negative number something's wrong.
        return testcall(a, b);
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"
    JNIEXPORT jlong JNICALL Java_org_ftc9974_thorcore_vision_NEONVision_processARGB(
            JNIEnv* env, jobject obj, jobject jImageBuffer, jlong bufferLen, jlong numPixels,
            jint lowA, jint highA, jint lowR, jint highR, jint lowG, jint highG, jint lowB, jint highB) {
        auto* bufferPtr = (uint8_t*) env->GetDirectBufferAddress(jImageBuffer);
        if (bufferPtr == nullptr) {
            return -1;
        }

        uint32_t pixel = *bufferPtr << 24 |
                *(bufferPtr + 1) << 16 |
                *(bufferPtr + 2) << 8 |
                *(bufferPtr + 3);
        __android_log_print(ANDROID_LOG_INFO, TAG, "pixel: %x -> %d, %d, %d, %d",
                pixel, *bufferPtr, *(bufferPtr + 1), *(bufferPtr + 2), *(bufferPtr + 3));

        uint64_t high = highA;
        high <<= 8;
        high |= highR;
        high <<= 8;
        high |= highG;
        high <<= 8;
        high |= highB;
        uint64_t low = lowA;
        low <<= 8;
        low |= lowR;
        low <<= 8;
        low |= lowG;
        low <<= 8;
        low |= lowB;
        uint64_t ret = accumThreshold(bufferPtr, bufferLen, &high, &low, numPixels);
        return ret;
    }

    // note that this doesn't do padding, since (for now) i'm only using it with a 16:9 aspect ratio
    JNIEXPORT jlong JNICALL Java_org_ftc9974_thorcore_vision_NEONVision_processPointerARGB(
            JNIEnv* env, jobject obj, jlong bufferPtr, jlong bufferLen, jint highColor, jint lowColor) {
        auto* pixels = reinterpret_cast<uint8_t*>(bufferPtr);
        uint64_t pixelBufLen = bufferLen;
        uint64_t high = highColor;
        uint64_t low = lowColor;
        return accumThreshold(pixels, pixelBufLen, &high, &low, pixelBufLen >> 2);
    }
#pragma clang diagnostic pop*/
}
