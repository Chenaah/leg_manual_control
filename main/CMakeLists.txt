set(srcs
        "main.c"
        "sketch.cpp")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "MotorCtrl" "tla20XX")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
