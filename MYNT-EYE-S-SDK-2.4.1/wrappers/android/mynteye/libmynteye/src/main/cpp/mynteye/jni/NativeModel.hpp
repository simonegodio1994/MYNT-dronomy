// AUTOGENERATED FILE - DO NOT MODIFY!
// This file generated by Djinni from mynteye_types.djinni

#pragma once

#include "djinni_support.hpp"
#include "model.hpp"

namespace djinni_generated {

class NativeModel final : ::djinni::JniEnum {
public:
    using CppType = ::mynteye_jni::Model;
    using JniType = jobject;

    using Boxed = NativeModel;

    static CppType toCpp(JNIEnv* jniEnv, JniType j) { return static_cast<CppType>(::djinni::JniClass<NativeModel>::get().ordinal(jniEnv, j)); }
    static ::djinni::LocalRef<JniType> fromCpp(JNIEnv* jniEnv, CppType c) { return ::djinni::JniClass<NativeModel>::get().create(jniEnv, static_cast<jint>(c)); }

private:
    NativeModel() : JniEnum("com/slightech/mynteye/Model") {}
    friend ::djinni::JniClass<NativeModel>;
};

}  // namespace djinni_generated
