find_package (Boost COMPONENTS filesystem program_options REQUIRED)

add_executable(${PROJECT_NAME}
        main.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE
        probeoptimizer
        Boost::filesystem
        Boost::program_options
)

add_executable(run_comparison
        run_comparison.cpp
)
target_link_libraries(run_comparison PRIVATE
        probeoptimizer
        Boost::filesystem
        Boost::program_options
)

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES
        ${CMAKE_SOURCE_DIR}/sites.csv
        ${CMAKE_SOURCE_DIR}/sample-inventory.csv
        DESTINATION
        ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/
)
