cmake_minimum_required(VERSION 2.8.3)

# Make the targets here.
add_custom_target(
  git-clone-json-c ALL
)

add_custom_command(
  TARGET git-clone-json-c PRE_BUILD
  COMMAND ${PROJECT_SOURCE_DIR}/3rdparty/clone-git.sh https://github.com/json-c/json-c.git ${PROJECT_SOURCE_DIR}/3rdparty/json-c
  COMMAND ${PROJECT_SOURCE_DIR}/3rdparty/build-json-c.sh ${PROJECT_SOURCE_DIR}/3rdparty/json-c
)
