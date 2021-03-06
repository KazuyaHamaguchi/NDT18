// ROSメインヘッダーファイル
// ROSプログラミングを行う際に必要となるROSファイルのインクルードを行う。
// 後述するROS_INFO関数などを使用できるようになる。
#include "ros/ros.h"
// srvTutorialサービスファイルのヘッダー
// CMakelists.txtでビルド後に自動的に生成されるように設定したサービスファ
// イルのヘッダーをインクルードする。
#include "arduino_throw_on/throw_on.h"
// atoll関数を使用するためのライブラリ
#include <cstdlib>
// サービスクライアントノードのメイン関数
int main(int argc, char **argv)
{
	// ノード名の初期化
	ros::init(argc, argv, "throw_on_service_client");
	// 入力値エラー処理
	if(argc != 2)
	{
		ROS_INFO("cmd: rosrun arduino_throw_on throw_on_service_client arg1");
		ROS_INFO( "arg1: double number");
		return 1;
	}
	// ROSシステムとの通信のためのノードのハンドル宣言
	ros::NodeHandle nh;
	// サービスクライアント宣言、
	// arduino_serviceパッケージのsrvTutorialサービスファイルを利用した。
	// サービスクライアントarduino_service_clientを作成する。
	// サービス名は「Throw_on」である。
	ros::ServiceClient throw_on_service_client =
	nh.serviceClient <arduino_throw_on::throw_on>("Throw_on_1");
	// srvという名前でsrvTutorialサービスを利用する
	// サービスのファイルを宣言する。
	arduino_throw_on::throw_on test;
	// サービスリクエスト値をそれぞれのa、bに格納する。
	test.request.input = atoll(argv [1]);
	// サービスをリクエストし、レスポンスが返された場合、
	// レスポンス値を表示する。
	if(throw_on_service_client.call(test))
	{
		ROS_INFO("send srv.request.input: %f",
		test.request.input);
		ROS_INFO("receive srv.response.output: %f",
		test.response.output);
	}
	else
	{
		ROS_ERROR("Failed to call service Throw_on_1");
		return 1;
	}
	return 0;
}
