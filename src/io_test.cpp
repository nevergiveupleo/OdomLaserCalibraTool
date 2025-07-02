#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <io.h>
#include <scan_match.h>
#include <solver.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("calib_io");
  messageIO dataIO(node);
  cScanMatch cScan;
  cSynchronizer cSync;
  cSolver cSolve;
//  std::string file_path = "/home/wangzhijie/Downloads/dataset/";
//  std::string bagname = "2018-12-28-15-16-07";
  std::string file_path;
  std::string bagname;
  std::string laser_topic;
  std::string odom_topic;

  node->declare_parameter<std::string>("file_path", "");
  node->declare_parameter<std::string>("bagname", "");
  node->declare_parameter<std::string>("laser_topic", "/scan");
  node->declare_parameter<std::string>("odom_topic", "/odom");

  node->get_parameter("file_path", file_path);
  node->get_parameter("bagname", bagname);
  node->get_parameter("laser_topic", laser_topic);
  node->get_parameter("odom_topic", odom_topic);

  std::string bagfile = file_path + bagname;
  std::string resultfile = file_path + bagname + "plicp_results.txt";
  std::string syncfile = file_path + bagname + "sync_results.txt";
  std::string smfile = file_path + bagname + "scan_match_results.txt";
  std::string odomfile = file_path + bagname + "odom.txt";

//  std::string laser_topic = "/scan";
//  std::string odom_topic = "/odom";
  std::vector<messageIO::laserScanData> laser_data(0);
  std::vector<messageIO::odometerData> odom_data(0);

  dataIO.readDataFromBag(bagfile, laser_topic, odom_topic, odom_data, laser_data);

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Hi!", WHITE, REGULAR).c_str());
  RCLCPP_INFO(node->get_logger(), "Laser size: %zu", laser_data.size());
  RCLCPP_INFO(node->get_logger(), "Odom size: %zu", odom_data.size());

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Begin CSM!", GREEN, REGULAR).c_str());
  std::vector<cScanMatch::csm_odom> scan_odom(0);
  std::vector<LDP> ldp(0);
  std::vector<cScanMatch::csm_results> match_results(0);

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Converting scan to csm type...", YELLOW, REGULAR).c_str());
  cScan.scanToLDP(laser_data, ldp);
  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Converting finished!", GREEN, REGULAR).c_str());

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("LDP numbers: ", WHITE, REGULAR).c_str());

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Canonical scan matching...", YELLOW, REGULAR).c_str());
  cScan.match(laser_data, ldp, scan_odom, match_results);
  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Matching finished!", GREEN, REGULAR).c_str());

  if (scan_odom.size() == 0) {
    RCLCPP_INFO(node->get_logger(), "%s", colouredString("ERROR! No matching results!", RED, BOLD).c_str());
  }

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Writing results...", YELLOW, REGULAR).c_str());

  std::ofstream fout(resultfile);
  for (int i = 0; i < scan_odom.size(); i++)
  {
    fout << scan_odom[i].timestamp.nanoseconds() << ' ' << scan_odom[i].x << ' ' << scan_odom[i].y
         << ' ' << scan_odom[i].theta << '\n';
  }

  std::ofstream fout3(smfile);
  for (int i = 0; i < match_results.size(); i++)
  {
    fout3 << match_results[i].start_t.nanoseconds() << ' ' << match_results[i].end_t.nanoseconds() << ' '
          << match_results[i].T << ' ' << match_results[i].scan_match_results[0] << ' '
          << match_results[i].scan_match_results[1] << ' '
          << match_results[i].scan_match_results[2] << '\n';
  }

  std::ofstream fout4(odomfile);
  for (int i = 0; i < odom_data.size(); i ++)
  {
    fout4 << odom_data[i].timestamp.nanoseconds() << ' ' << odom_data[i].v_l << ' '
          << odom_data[i].v_r << '\n';
  }

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Saving results...", YELLOW, REGULAR).c_str());
  fout << std::flush;
  fout.close();
  fout3 << std::flush;
  fout3.close();
  fout4 << std::flush;
  fout4.close();

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Results saved!", GREEN, REGULAR).c_str());

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Start synchronzing...", YELLOW, REGULAR).c_str());
  std::vector<cSynchronizer::sync_data> sync_results;
  cSync.synchronizeLaserOdom(odom_data, match_results, sync_results);
  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Sync finished!", GREEN, REGULAR).c_str());

  std::ofstream fout2(syncfile);
  for (int i = 0; i < sync_results.size(); i++)
  {
    fout2 << sync_results[i].T << ' ' << sync_results[i].velocity_left << ' '
          << sync_results[i].velocity_right << ' '
          << sync_results[i].scan_match_results[0] << ' '
          << sync_results[i].scan_match_results[1] << ' '
          << sync_results[i].scan_match_results[2] << '\n';
  }
  fout2 << std::flush;
  fout2.close();

  RCLCPP_INFO(node->get_logger(), "%s", colouredString("Sync results saved!", GREEN, REGULAR).c_str());

  cSolve.calib(sync_results, 4);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
