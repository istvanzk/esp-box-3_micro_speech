cmake_minimum_required(VERSION 3.10)
set(IDF_TARGET "esp32s3")
set(ADF_PATH $ENV{ADF_PATH})
set(EXTRA_COMPONENT_DIRS
    ${ADF_PATH}/components
    test_data
    main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(micro_speech)
