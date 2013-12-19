rock_library(wbc
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(wbc_bin Main.cpp
    DEPS wbc)

