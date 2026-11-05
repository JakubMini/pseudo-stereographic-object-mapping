// The app's only door into the engine: paths in, a JSON result out.
#include <jni.h>

#include <string>

#include <android/log.h>
#include <nlohmann/json.hpp>

#include "psom/pipeline.h"

namespace {

std::string str(JNIEnv* env, jstring s) {
    const char* c = env->GetStringUTFChars(s, nullptr);
    std::string out(c);
    env->ReleaseStringUTFChars(s, c);
    return out;
}

jstring result(JNIEnv* env, const psom::Result& r) {
    const nlohmann::json j = {{"ok", r.ok}, {"stage", r.stage}, {"error", r.error}, {"warnings", r.warnings}};
    return env->NewStringUTF(j.dump().c_str());
}

psom::Progress progress(JNIEnv* env, jobject listener) {
    if (!listener) return {};
    jclass cls = env->GetObjectClass(listener);
    jmethodID m = env->GetMethodID(cls, "onProgress", "(Ljava/lang/String;D)V");
    return [env, listener, m](const std::string& stage, double f) {
        jstring s = env->NewStringUTF(stage.c_str());
        env->CallVoidMethod(listener, m, s, f);
        env->DeleteLocalRef(s);
    };
}

// A C++ exception must never cross into the JVM.
template <typename F>
jstring guarded(JNIEnv* env, const char* stage, F&& f) {
    try {
        return result(env, f());
    } catch (const std::exception& e) {
        __android_log_print(ANDROID_LOG_ERROR, "psom", "%s failed: %s", stage, e.what());
        return result(env, psom::Result::fail(stage, std::string("engine error: ") + e.what()));
    } catch (...) {
        return result(env, psom::Result::fail(stage, "unknown engine error"));
    }
}

}  // namespace

extern "C" JNIEXPORT jstring JNICALL Java_com_jakubmini_psom_engine_EngineBridge_reconstruct(JNIEnv* env, jobject, jstring dir,
                                                                                         jint mem_mb, jobject listener) {
    return guarded(env, "reconstruct", [&] {
        psom::Options o;
        o.mem_mb = size_t(mem_mb);
        return psom::reconstruct(str(env, dir), o, progress(env, listener));
    });
}

extern "C" JNIEXPORT jstring JNICALL Java_com_jakubmini_psom_engine_EngineBridge_export(JNIEnv* env, jobject, jstring dir,
                                                                                    jobject listener) {
    return guarded(env, "export", [&] { return psom::export_project(str(env, dir), progress(env, listener)); });
}
