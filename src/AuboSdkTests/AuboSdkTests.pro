TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

unix {

        #64bit OS
        contains(QT_ARCH, x86_64){

                INCLUDEPATH += $$PWD/dependents/inc

                LIBS += -L$$PWD/dependents/lib/ -laubo_sdk

                LIBS += -lpthread
    }
}

SOURCES += main.cpp

include(./examples/example.pri)

HEADERS +=
