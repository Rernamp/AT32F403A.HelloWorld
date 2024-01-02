target_sources(CommonCode INTERFACE
    main.cpp
    at32f403a_407_conf.h
)

target_include_directories(CommonCode INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
