#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <vector>

#include "raspicat_tvvf_navigation/waypoint_follower_node.hpp"
#include "raspicat_tvvf_navigation/command_utils.hpp"

using raspicat_tvvf_navigation::WaypointFollowerNode;

namespace {

geometry_msgs::msg::Pose makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;

  const double half = yaw * 0.5;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(half);
  pose.orientation.w = std::cos(half);
  return pose;
}

std::filesystem::path writeWaypointCsv(const std::string & filename, const std::vector<std::string> & rows)
{
  const auto csv_path = std::filesystem::temp_directory_path() / filename;
  std::ofstream ofs(csv_path);
  ofs << "id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command\n";
  for (const auto & row : rows) {
    ofs << row << "\n";
  }
  return csv_path;
}

}  // namespace

class TestWaypointFollowerNode : public WaypointFollowerNode
{
public:
  TestWaypointFollowerNode()
  : WaypointFollowerNode(), pose_call_count_(0)
  {
    // タイマーは起動しない想定（spinしないため）
  }

  bool loadWps(const std::string& path) { return loadWaypointsForTest(path); }
  void setState(raspicat_tvvf_navigation::NavigationState state) { setStateForTest(state); }
  raspicat_tvvf_navigation::NavigationState getState() const { return getStateForTest(); }
  void markReached() { markReachedForTest(); }
  void runOnce() { controlLoop(); }
  void setGoalTimeout(double timeout_sec) { setGoalTimeoutForTest(timeout_sec); }
  void setMaxRetryCount(int max_retry_count) { setMaxRetryCountForTest(max_retry_count); }

  void setTestPose(const std::optional<geometry_msgs::msg::Pose>& pose)
  {
    test_pose_ = pose;
  }

  int getPoseCallCount() const { return pose_call_count_; }

protected:
  std::optional<geometry_msgs::msg::Pose> getRobotPose() override
  {
    ++pose_call_count_;
    return test_pose_;
  }

private:
  std::optional<geometry_msgs::msg::Pose> test_pose_;
  int pose_call_count_;
};

TEST(WaypointFollowerNodeTest, UsesSinglePosePerControlLoop)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<TestWaypointFollowerNode>();

  // 簡易CSV
  std::ostringstream ss;
  ss << "id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command\n";
  ss << "0,0,0,0,0,0,0,1,\n";
  ss << "1,1,0,0,0,0,0.7071068,0.7071068,strict pause\n";
  const auto csv_path = std::filesystem::temp_directory_path() / "wf_single_pose.csv";
  {
    std::ofstream ofs(csv_path);
    ofs << ss.str();
  }

  // パラメータ設定と開始
  node->set_parameter(rclcpp::Parameter("waypoint_csv_path", csv_path.string()));
  ASSERT_TRUE(node->loadWps(csv_path.string()));
  node->setState(raspicat_tvvf_navigation::NavigationState::NAVIGATING);
  node->markReached();  // 先頭WPを到達扱いにしてstrict pauseのWPを現在地に

  // ゴール内の姿勢を設定して到達判定
  node->setTestPose(makePose(1.0, 0.0, M_PI_2));
  node->runOnce();  // NAVIGATING -> WAYPOINT_REACHED
  node->runOnce();  // execute command -> WAITING

  EXPECT_EQ(node->getPoseCallCount(), 1);  // NAVIGATINGループのみ姿勢取得
  EXPECT_EQ(node->getState(), raspicat_tvvf_navigation::NavigationState::WAITING);

  rclcpp::shutdown();
}

TEST(WaypointFollowerNodeTest, DoesNotFetchPoseInIdleState)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<TestWaypointFollowerNode>();
  node->setState(raspicat_tvvf_navigation::NavigationState::IDLE);

  node->runOnce();

  EXPECT_EQ(node->getPoseCallCount(), 0);

  rclcpp::shutdown();
}

TEST(WaypointFollowerNodeTest, PublishesGoalOnlyOnceUntilTimeoutRetry)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<TestWaypointFollowerNode>();
  auto csv_path = writeWaypointCsv(
    "wf_goal_once.csv",
    {
      "0,3.0,0.0,0,0,0,0,1,",
    });

  node->set_parameter(rclcpp::Parameter("waypoint_csv_path", csv_path.string()));
  node->set_parameter(rclcpp::Parameter("enable_skip_on_timeout", true));
  node->setGoalTimeout(0.05);
  node->setMaxRetryCount(2);
  ASSERT_TRUE(node->loadWps(csv_path.string()));
  node->setState(raspicat_tvvf_navigation::NavigationState::NAVIGATING);
  node->setTestPose(makePose(0.0, 0.0, 0.0));

  auto receiver = std::make_shared<rclcpp::Node>("wf_goal_counter");
  int goal_count = 0;
  auto sub = receiver->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    [&](const geometry_msgs::msg::PoseStamped &) {
      ++goal_count;
    });
  (void)sub;

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(receiver);

  node->runOnce();
  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  node->runOnce();
  exec.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  node->runOnce();
  exec.spin_some();

  // goal_timeout(50ms)未満では再送しない
  EXPECT_EQ(goal_count, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  node->runOnce();
  exec.spin_some();

  // timeout後はretryで再送する
  EXPECT_EQ(goal_count, 2);

  exec.remove_node(receiver);
  exec.remove_node(node);
  rclcpp::shutdown();
}
