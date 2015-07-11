@[if DEVELSPACE]@
set(_meshes
  @(CMAKE_CURRENT_SOURCE_DIR)/meshes
)
set(_materials
  @(CMAKE_CURRENT_SOURCE_DIR)/materials
)
@[else]@
set(_meshes
  ${r2_description_DIR}/../../../@(CATKIN_PACKAGE_SHARE_DESTINATION)/meshes
)
set(_materials
  ${r2_description_DIR}/../../../@(CATKIN_PACKAGE_SHARE_DESTINATION)/materials
)
@[end if]@

function(r2_description_symlink_resources name output)
  message(" linking: "${_meshes})
  message("      to: "${output}/meshes)
  message(" linking: "${_materials})
  message("      to: "${output}/materials)
  add_custom_command(OUTPUT ${name} COMMAND ln -sf ${_meshes} ${output}
                                    COMMAND ln -sf ${_materials} ${output})
endfunction(r2_description_symlink_resources)

