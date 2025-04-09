
QT += core gui widgets charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    exportdialog.cpp \
    generator.cpp \
    simulation.cpp \
    arx.cpp \
    chartwidget.cpp \
    arxchangeparameters.cpp \
    pid.cpp

HEADERS += \
    mainwindow.h \
    exportdialog.h \
    generator.h \
    simulation.h \
    arx.h \
    chartwidget.h \
    arxchangeparameters.h \
    pid.h

FORMS += \
    mainwindow.ui \
    exportdialog.ui \
    arxchangeparameters.ui \
    chartwidget.ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
