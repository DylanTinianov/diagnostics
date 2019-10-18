// copyright Copyright (c) 2019, Clearpath Robotics, Inc., All rights reserved.

#include "diagnostic_aggregator/folding_analyzer.h"
#include <string>
#include <vector>

using std::vector;
using std::string;

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::FoldingAnalyzer,
                       diagnostic_aggregator::Analyzer)


using namespace diagnostic_aggregator;

FoldingAnalyzer::FoldingAnalyzer() :
    path_(""), nice_name_("Folding"), has_initialized_(false)
{}

FoldingAnalyzer::~FoldingAnalyzer()
{}

bool FoldingAnalyzer::init(const std::string base_name, const ros::NodeHandle &n)
{
  path_ = base_name + "/" + nice_name_;

  if (!n.getParam("top_level", top_level_))
  {
    ROS_ERROR("No top_level was specified in yaml file for Folding Analyzer");
    return false;
  }

  string fl;
  if (!n.getParam("fold_level", fl))
  {
    fold_level_ = diagnostic_msgs::DiagnosticStatus::ERROR;
  }
  else
  {
    if (fl== "2")
    {
      fold_level_ = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    else if (fl == "1")
    {
      fold_level_ = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    else if (fl == "0")
    {
      fold_level_ = diagnostic_msgs::DiagnosticStatus::OK;
    }
    else
    {
      ROS_ERROR("Invalid fold_level. fold_level must be [0, 1, 2]. Check Yaml file.");
      return false;
    }
  }

  if (!n.getParam("fold", fold))
  {
    ROS_ERROR("No fold configurations were specified for the top_level in yaml file for Folding Analyzer");
    return false;
  }

  boost::shared_ptr<StatusItem> top_item(new StatusItem(""));
  boost::shared_ptr<StatusItem> item(new StatusItem(""));

  top_level_item_ = top_item;
  current_item_ = item;

  has_initialized_ = true;
  return true;
}


bool FoldingAnalyzer::match(const std::string name)
{
  if (name == top_level_)
  {
    return true;
  }

  for (int i=0; i < fold.size(); i++)
  {
    if (name == fold[i])
    {
      return true;
    }
  }

  return false;
}


bool FoldingAnalyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
  if (item->getName() == top_level_)
  {
    top_level_item_ = item;
    return false;
  }

  for (int i=0; i < fold.size(); i++)
  {
    if (item->getName().find(fold[i]) != string::npos)
    {
      current_item_ = item;
      return true;
    }
  }

  return false;
}


vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> FoldingAnalyzer::report()
{
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> curr_stat = current_item_->toStatusMsg(path_);
  if (top_level_item_->getLevel() == fold_level_)
  {
    curr_stat->level = diagnostic_msgs::DiagnosticStatus::OK;

    curr_stat->message = "SUPPRESSED by " + top_level_ + " Message:  " + curr_stat->message;

    // Add key value to show that the error has been folded
    diagnostic_msgs::KeyValue kv;
    kv.key = "Folded";
    kv.value = "true";
    curr_stat->values.push_back(kv);
  }

  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;
  output.push_back(curr_stat);

  return output;
}

