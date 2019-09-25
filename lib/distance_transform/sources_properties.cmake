function(set_distance_transform_source_files_properties)
    set_dope_vector_source_files_properties()
    get_target_property(files distance_transform INTERFACE_SOURCES)
    foreach(file IN LISTS files)
        if (file MATCHES "[.]*.inl")
            set_source_files_properties(${file} PROPERTIES XCODE_EXPLICIT_FILE_TYPE "sourcecode.cpp.h")
        endif()
        get_filename_component(file_path_dir ${file} DIRECTORY)
        if (file_path_dir MATCHES "[.]*[/\\]distance_transform[/\\]inlines")
            source_group("distance_transform\\inlines" FILES ${file})
        elseif(file_path_dir MATCHES "[.]*[/\\]distance_transform")
            source_group("distance_transform" FILES ${file})
        endif()
    endforeach()
endfunction(set_distance_transform_source_files_properties)
