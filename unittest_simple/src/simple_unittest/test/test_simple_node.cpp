
#include "simple_unittest/simple_node.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");

    // テスト対象のノード
    test_node_ = std::make_shared<SimpleNode>();

    // パブリッシャー（入力用）
    publisher_ = node_->create_publisher<std_msgs::msg::String>("input_topic", 10);

    // サブスクライバー（出力用）
    subscription_ = node_->create_subscription<std_msgs::msg::String>("output_topic", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
      received_msg_ = msg;
      received_ = true;
    });

    received_ = false;
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<SimpleNode> test_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std_msgs::msg::String::SharedPtr received_msg_;
  bool received_;
};

TEST_F(SimpleNodeTest, MessageProcessingTest) {
  auto message = std::make_shared<std_msgs::msg::String>();
  message->data = "Hello";

  // メッセージをパブリッシュ
  publisher_->publish(*message);

  // ノードを一定時間スピンしてコールバックを実行
  auto start = std::chrono::steady_clock::now();
  while (!received_ && std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    rclcpp::spin_some(test_node_);
    rclcpp::spin_some(node_);
  }

  // メッセージが正しく受信されたか確認
  ASSERT_TRUE(received_);
  EXPECT_EQ(received_msg_->data, "Processed: Hello");
}
