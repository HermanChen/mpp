# vim: syntax=cmake

# hal vp9 decoder sourse
set(HAL_VP9D_SRC
    hal_vp9d_api.c
    hal_vp9d_com.c
    hal_vp9d_rkv.c
    hal_vp9d_vdpu34x.c
    hal_vp9d_vdpu382.c
    hal_vp9d_vdpu383.c
    )

add_library(hal_vp9d STATIC ${HAL_VP9D_SRC})

target_link_libraries(hal_vp9d mpp_base vdpu383_com)
set_target_properties(hal_vp9d PROPERTIES FOLDER "mpp/hal")

