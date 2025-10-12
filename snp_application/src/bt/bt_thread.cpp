#include <snp_application/bt/bt_thread.h>
#include <chrono>
#include <thread>

namespace snp_application
{
BTThread::BTThread(QObject* parent) : QThread(parent), tree({}), result(BT::NodeStatus::IDLE)
{
}

BTThread::BTThread(BT::Tree tree, QObject* parent)
  : QThread(parent), tree(std::move(tree)), result(BT::NodeStatus::IDLE)
{
}

BTThread::BTThread(BT::Tree tree, unsigned groot2_port, QObject* parent)
  : QThread(parent), tree(std::move(tree)), result(BT::NodeStatus::IDLE)
{
  // Initialize Groot2 publisher for real-time monitoring
  // Add a small delay to allow previous instance to clean up
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  groot2_publisher = std::make_shared<BT::Groot2Publisher>(this->tree, groot2_port);
}

BTThread::~BTThread()
{
  // Ensure thread is finished before destructing
  if (isRunning()) {
    quit();
    wait();
  }
  
  // Explicitly reset the Groot2Publisher to ensure proper cleanup
  if (groot2_publisher) {
    groot2_publisher.reset();
  }
}

void BTThread::run()
{
  try
  {
    result = tree.tickWhileRunning();
  }
  catch (const std::exception& ex)
  {
    message = ex.what();
    result = BT::NodeStatus::FAILURE;
  }
}

}  // namespace snp_application
