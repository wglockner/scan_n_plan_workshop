#pragma once

#include <QThread>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

namespace snp_application
{
/**
 * @brief Thread used to tick a behavior tree
 */
class BTThread : public QThread
{
public:
  BTThread(QObject* parent = nullptr);
  BTThread(BT::Tree tree, QObject* parent = nullptr);
  BTThread(BT::Tree tree, unsigned groot2_port, QObject* parent = nullptr);
  ~BTThread();

  BT::Tree tree;
  BT::NodeStatus result;
  QString message;
  std::shared_ptr<BT::Groot2Publisher> groot2_publisher;

protected:
  void run() override;
};

}  // namespace snp_application
