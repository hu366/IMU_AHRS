idf_component_register(SRCS "vqf.c"
                    INCLUDE_DIRS "include")

target_compile_definitions(${COMPONENT_LIB} PUBLIC VQF_SINGLE_PRECISION)
