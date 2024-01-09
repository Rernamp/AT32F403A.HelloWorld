target_sources(CommonCode INTERFACE
    at32f403a_407_clock.c
    at32f403a_407_clock.h
    main.cpp
    at32f403a_407_conf.h
    at32f403a_407_int.c
    at32f403a_407_int.h
    main.c
)

target_include_directories(CommonCode INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
