cmake_minimum_required(VERSION 3.16)
project(m5_hardware)

# モダンなC++設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依存パッケージを自動で処理
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ライブラリの定義（.so を作成、インストールまで一括）
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/m5_hardware.cpp
)

# pluginlib のエクスポート（plugin.xml → m5_hardware.xml）
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

# ヘッダファイルなどのインストール
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 自動 install がうまく動かない場合に備えて保険を掛けたいなら:
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)


# エクスポート（他パッケージで使う場合に備えて）
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

