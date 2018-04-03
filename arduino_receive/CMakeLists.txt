cmake_minimum_required(VERSION 2.8.3)
project(arduino_receive)
## catkinビルドに必要なコンポーネントパッケージの設定
## これらのパッケージが存在しない場合、catkinビルドの途中でエラーが出る。
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
## メッセージファイルの設定
##add_message_files(FILES msgTutorial.msg)
## サービスファイルの設定
add_service_files(FILES receive.srv)
## add_message_filesで使用するメッセージの依存関係を設定
## これらのパッケージが存在しない場合、catkinビルドの途中でエラーが出る。
generate_messages(DEPENDENCIES std_msgs)
## インクルードディレクトリ、ライブラリ、catkinビルド、システムに依存するパッケージの指定
catkin_package(
#INCLUDE_DIRS include
LIBRARIES arduino_receive
CATKIN_DEPENDS roscpp std_msgs
DEPENDS system_lib
)
## インクルードディレクトリの設定
include_directories(include ${catkin_INCLUDE_DIRS})
## arduino_service_srv_clientサービスクライアントノードの設定
## 実行ファイル、ターゲットリンクライブラリ、追加の依存関係などを設定
add_executable(receive_service_client src/receive_service_client.cpp)
target_link_libraries(receive_service_client ${catkin_LIBRARIES})
add_dependencies(receive_service_client receive_service_generate_messages_cpp)
