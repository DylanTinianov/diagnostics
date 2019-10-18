// copyright Copyright (c) 2019, Clearpath Robotics, Inc., All rights reserved.
#ifndef FOLDING_ANALYZER_FOLDING_ANALYZER_H
#define FOLDING_ANALYZER_FOLDING_ANALYZER_H

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>

namespace diagnostic_aggregator
{

class FoldingAnalyzer : public Analyzer
{
public:
  FoldingAnalyzer();

  ~FoldingAnalyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }

  std::string getName() const { return nice_name_; }

private:
  std::string top_level_;
  std::vector<std::string> fold;  // List of components to fold

  // Store status items for folding
  boost::shared_ptr<StatusItem> top_level_item_;
  boost::shared_ptr<StatusItem> current_item_;

  int fold_level_;
  std::string path_, nice_name_;

  bool has_initialized_;
};

}  // namespace diagnostic_aggregator

#endif  // FOLDING_ANALYZER_FOLDING_ANALYZER_H
