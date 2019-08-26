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
    set(oneValueArgs NAME VERSION INCLUDE_ROOT_DIR INCLUDE_INSTALL_DIR INCLUDE_SOURCE_DIR EXAMPLE_INSTALL_DIR RELOCATABLE)
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

    sofa_install_targets("${ARG_NAME}" "${ARG_TARGETS}" "${include_install_dir}" "${ARG_INCLUDE_SOURCE_DIR}" "${ARG_EXAMPLE_INSTALL_DIR}" "${ARG_RELOCATABLE}")
    sofa_write_package_config_files("${ARG_NAME}" "${ARG_VERSION}")
endfunction()


macro(iab_add_plugin directory plugin_name)
    iab_add_generic( ${directory} ${plugin_name} "Plugin" ${ARGV2} )
    message(STATUS "Added ${plugin_name} at path: ${directory} to list of plugins")
endmacro()
