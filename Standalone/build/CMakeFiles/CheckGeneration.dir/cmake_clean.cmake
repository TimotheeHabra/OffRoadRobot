FILE(REMOVE_RECURSE
  "CMakeFiles/CheckGeneration"
  "../src/project/controller_files/ControllersStruct.h"
  "../src/project/controller_files/ControllersStruct.c"
  "../src/project/user_files/user_sf_IO.h"
  "../src/project/user_files/user_sf_IO.c"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/CheckGeneration.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
