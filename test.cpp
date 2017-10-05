#include <boost/program_options.hpp>
#include <is/is.hpp>
#include "msgs/robot-controller.hpp"
#include "task.hpp"

namespace po = boost::program_options;
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string topic;
  std::string filename;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://edge.is:30000"), "broker uri");
  options("topic,t", po::value<std::string>(&topic)->default_value("robot-controller.0.do-task"),
          "topic to request robot task");
  options("filename,f", po::value<std::string>(&filename), "yaml file with task parameters");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("filename")) {
    std::cout << description << std::endl;
    return 1;
  }

  auto robot_task = task::from_file(filename);

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  auto id = client.request(topic, is::msgpack(robot_task));
  auto reply = client.receive_for(5s, id, is::policy::discard_others);
  if (reply == nullptr) {
    is::log::warn("Can't request task");
  } else {
    is::log::info("Task requested");
  }
  return 0;
}