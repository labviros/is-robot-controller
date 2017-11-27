#include <boost/program_options.hpp>
#include <is/msgs/common.pb.h>
#include <is/msgs/robot.pb.h>
#include <is/is.hpp>
// #include "task.hpp"

using namespace is::common;
using namespace is::robot;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string topic;
  std::string filename;

  is::po::options_description opts("Options");
  auto&& opt_add = opts.add_options();

  opt_add("help,", "show available opt_add");
  opt_add("uri,u", is::po::value<std::string>(&uri)->default_value("amqp://rmq.is:30000"), "broker uri");
  opt_add("topic,t", is::po::value<std::string>(&topic)->default_value("RobotController.0.SetTask"),
          "topic to request robot task");
  // opt_add("filename,f", is::po::value<std::string>(&filename)->required(), "yaml file with task parameters");

  is::parse_program_options(argc, argv, opts);

  // auto robot_task = task::from_file(filename);
  RobotTask robot_task;
  robot_task.mutable_pose()->mutable_goal()->mutable_position()->set_x(1.0 /* meters */);
  robot_task.mutable_pose()->mutable_goal()->mutable_position()->set_y(1.0 /* meters */);
  robot_task.mutable_sampling()->set_frequency(10.0);
  robot_task.set_allowed_error(0.2 /* meters */);

  robot_task.PrintDebugString();

  is::info("Trying to connect to {}", uri);
  auto channel = is::rmq::Channel::CreateFromUri(uri);
  auto tag = is::declare_queue(channel);
 
  auto msg = is::pack_proto(robot_task);
  msg->ReplyTo(tag);

  channel->BasicPublish("is", topic, msg);
  
  is::info("Waiting for reply");
  auto envelope = channel->BasicConsumeMessage(tag);
  auto status = is::rpc_status(envelope);

  is::info("{} | {}", StatusCode_Name(status.code()), status.why());

  return 0;
}