
macro(iab_add_plugin directory plugin_name)
    iab_add_generic( ${directory} ${plugin_name} "Plugin" ${ARGV2} )
    message(STATUS "Added ${plugin_name} at path: ${directory} to list of plugins")
endmacro()

macro(iab_add_generic directory name type)
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${directory}" AND IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${directory}")
        string(TOUPPER ${type}_${name} option)

        # optional parameter to activate/desactivate the option
        #  e.g.  sofa_add_application( path/MYAPP MYAPP APPLICATION ON)
        set(active OFF)
        if(${ARGV3})
            if( ${ARGV3} STREQUAL ON )
                set(active ON)
            endif()
        endif()

        option(${option} "Build the ${name} ${type}." ${active})
        if(${option})
            message("Adding ${type} ${name}")
            add_subdirectory(${directory} ${name})
            #Check if the target has been successfully added
            if(TARGET ${name})
                set_target_properties(${name} PROPERTIES FOLDER ${type}s) # IDE folder
                set_target_properties(${name} PROPERTIES DEBUG_POSTFIX "_d")
            endif()
        endif()

        # Add current target in the internal list only if not present already
        get_property(_allTargets GLOBAL PROPERTY __GlobalTargetList__)
        get_property(_allTargetNames GLOBAL PROPERTY __GlobalTargetNameList__)

        # if(NOT ${name} IN_LIST _allTargets) # ONLY CMAKE >= 3.3 and policy to NEW
        list (FIND _allTargets ${name} _index)
        if(NOT ${_index} GREATER -1)
            set_property(GLOBAL APPEND PROPERTY __GlobalTargetList__ ${name})
        endif()

        #if(NOT ${option} IN_LIST _allTargetNames)# ONLY CMAKE >= 3.3 and policy to NEW
        list (FIND _allTargetNames ${option} _index)
        if(NOT ${_index} GREATER -1)
            set_property(GLOBAL APPEND PROPERTY __GlobalTargetNameList__ ${option})
        endif()
    else()
        message("${type} ${name} (${CMAKE_CURRENT_LIST_DIR}/${directory}) does not exist and will be ignored.")
    endif()
endmacro()

function(iab_generate_package)
    set(oneValueArgs NAME VERSION INCLUDE_ROOT_DIR INCLUDE_INSTALL_DIR INCLUDE_SOURCE_DIR IAB_PLUGIN_INSTALL_DIR RELOCATABLE)
    set(multiValueArgs TARGETS)
    cmake_parse_arguments("ARG" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    # Required arguments
    foreach(arg ARG_NAME ARG_VERSION ARG_TARGETS)
        if("${${arg}}" STREQUAL "")
            string(SUBSTRING "${arg}" 4 -1 arg_name)
            message(SEND_ERROR "Missing parameter ${arg_name}.")
        endif()
    endforeach()

    set(include_install_dir "${ARG_INCLUDE_INSTALL_DIR}")
    if(NOT ARG_INCLUDE_INSTALL_DIR)
        if(ARG_INCLUDE_ROOT_DIR)
            set(include_install_dir "${ARG_INCLUDE_ROOT_DIR}")
            message(WARNING "iab_generate_package(${ARG_NAME}): INCLUDE_ROOT_DIR is deprecated. Please use INCLUDE_INSTALL_DIR instead.")
        else()
            set(include_install_dir "${ARG_NAME}")
        endif()
    endif()

    iab_install_targets("${ARG_NAME}" "${ARG_TARGETS}" "${include_install_dir}" "${ARG_INCLUDE_SOURCE_DIR}" "${ARG_EXAMPLE_INSTALL_DIR}" "${ARG_RELOCATABLE}")
    iab_write_package_config_files("${ARG_NAME}" "${ARG_VERSION}")
endfunction()

macro(iab_install_targets package_name the_targets include_install_dir)
    install(TARGETS ${the_targets}
            EXPORT ${package_name}Targets
            RUNTIME DESTINATION "bin" COMPONENT applications
            LIBRARY DESTINATION "lib" COMPONENT libraries
            ARCHIVE DESTINATION "lib" COMPONENT libraries
            PUBLIC_HEADER DESTINATION "include/${include_install_dir}" COMPONENT headers

            # [MacOS] install runSofa above the already populated runSofa.app (see CMAKE_INSTALL_PREFIX)
            BUNDLE DESTINATION "." COMPONENT applications
            )

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in")
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in" "${CMAKE_BINARY_DIR}/include/${package_name}/config.h")
        install(FILES "${CMAKE_BINARY_DIR}/include/${package_name}/config.h" DESTINATION "include/${include_install_dir}")
    endif()

    foreach(target ${the_targets})
        set(version ${${target}_VERSION})
        if(version VERSION_GREATER "0.0")
            set_target_properties(${target} PROPERTIES VERSION "${version}")
        elseif(target MATCHES "^IAB" AND SofaFramework_VERSION)
            set_target_properties(${target} PROPERTIES VERSION "${SofaFramework_VERSION}")
        elseif(target MATCHES "^IAB" AND Sofa_VERSION)
            set_target_properties(${target} PROPERTIES VERSION "${Sofa_VERSION}")
        endif()

        # non-flat headers install (if no PUBLIC_HEADER and include_install_dir specified)
        get_target_property(public_header ${target} PUBLIC_HEADER)
        if("${public_header}" STREQUAL "public_header-NOTFOUND" AND NOT "${include_install_dir}" STREQUAL "")
            set(optional_argv3 "${ARGV3}")
            if(optional_argv3)
                # ARGV3 is a non-breaking additional argument to handle INCLUDE_SOURCE_DIR (see sofa_generate_package)
                # TODO: add a real argument "include_source_dir" to this macro
                set(include_source_dir "${ARGV3}")
            endif()
            if(NOT EXISTS "${include_source_dir}" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${include_source_dir}")
                # will be true if include_source_dir is empty
                set(include_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/${include_source_dir}")
            endif()
            #message("${target}: ${include_source_dir} -> include/${include_install_dir}")
            file(GLOB_RECURSE header_files "${include_source_dir}/*.h" "${include_source_dir}/*.inl")
            foreach(header ${header_files})
                file(RELATIVE_PATH path_from_package "${include_source_dir}" "${header}")
                get_filename_component(dir_from_package ${path_from_package} DIRECTORY)
                install(FILES ${header}
                        DESTINATION "include/${include_install_dir}/${dir_from_package}"
                        COMPONENT headers)
            endforeach()
        endif()
    endforeach()

    ## Default install rules for resources
    set(example_install_dir "share/sofa/examples/${package_name}")
    set(optional_argv4 "${ARGV4}")
    if(optional_argv4)
        # ARGV3 is a non-breaking additional argument to handle EXAMPLE_INSTALL_DIR (see sofa_generate_package)
        # TODO: add a real argument "example_install_dir" to this macro
        set(example_install_dir "${optional_argv4}")
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
        install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples/" DESTINATION "${example_install_dir}" COMPONENT resources)
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scenes")
        install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scenes/" DESTINATION "${example_install_dir}" COMPONENT resources)
    endif()

    # RELOCATABLE optional arg
    set(optional_argv5 "${ARGV5}")
    if(optional_argv5)
        sofa_set_install_relocatable(${package_name} ${optional_argv5})
    endif()
endmacro()
