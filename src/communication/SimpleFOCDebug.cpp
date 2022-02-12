
#include "SimpleFOCDebug.h"

#ifndef SIMPLEFOC_DISABLE_DEBUG


Print* SimpleFOCDebug::_debugPrint = NULL;


void SimpleFOCDebug::enable(Print* debugPrint) {
    _debugPrint = debugPrint;
}



void SimpleFOCDebug::println(const char* str) {
    if (_debugPrint != NULL) {
        _debugPrint->println(str);
    }
}

void SimpleFOCDebug::println(const __FlashStringHelper* str) {
    if (_debugPrint != NULL) {
        _debugPrint->println(str);
    }
}

void SimpleFOCDebug::println(const char* str, float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

void SimpleFOCDebug::println(const __FlashStringHelper* str, float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

void SimpleFOCDebug::println(const char* str, int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

void SimpleFOCDebug::println(const __FlashStringHelper* str, int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

#endif