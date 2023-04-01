#include <boost/program_options.hpp>
#include <iostream>
#include <numeric>

#include "proto/parameters.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/constants.h"
#include "software/networking/threaded_proto_udp_listener.hpp"

/*
 * This standalone program listens for RobotLog protos on the specified ip address
 * and logs them
 */
// TODO: Switch robotCommunication multicast channel for robotstatus + logs + world + primitiveset
void logFromNetworking(TbotsProto::RobotLog log)
{
//    if (log.robot_id() != 5) {
//        return;
//    }
    LEVELS level(INFO);

    if (TbotsProto::LogLevel_Name(log.log_level()) == "DEBUG")
    {
        level = DEBUG;
    }
    else if (TbotsProto::LogLevel_Name(log.log_level()) == "WARNING" ||
             TbotsProto::LogLevel_Name(log.log_level()) == "FATAL")
    {
        // log FATAL as WARNING to prevent program from exiting
        // note that the level of the RobotLog will itself be printed
        level = WARNING;
    }

    LOG(level) << "[ROBOT " << log.robot_id() << " " << LogLevel_Name(log.log_level())
               << "]"
               << "[" << log.file_name() << ":" << log.line_number()
               << "] " << log.created_timestamp().epoch_timestamp_seconds() << ": " << log.log_msg();// << std::endl;
}

int main(int argc, char **argv)
{
    struct CommandLineArgs
    {
        bool help = false;
        std::string interface;
//        int channel = 0;
//        std::vector<int> connected_robots;
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("interface",
                       boost::program_options::value<std::string>(&args.interface),
                       "Which network interface to listen for messages from");
//    desc.add_options()("channel",
//                       boost::program_options::value<int>(&args.channel),
//                       "Multicast channel to listen on connect to");
//    desc.add_options()("connected_robots",
//                       boost::program_options::value<std::vector<int>>()->multitoken(),
//                       "Robots to show logs from. If empty, logs from all robots are shown");

//    boost::program_options::variables_map vm;
//    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
//    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
        return 0;
    }

//    if (vm["interface"] == "")
//    {
//        LOG(FATAL) << "No interface was provided. Run 'ifconfig' and choose an appropriate network interface";
//    }

//    if (!vm["connected_robots"].empty())
//    {
//        args.connected_robots = vm["connected_robots"].as<std::vector<int>>();
//    }
//
//    std::cout << "Connected: " << std::endl;
//    for (auto i : args.connected_robots)
//    {
//        std::cout << args.connected_robots[i] << std::endl;
//    }

    auto logWorker               = g3::LogWorker::createLogWorker();
    auto colour_cout_sink_handle = logWorker->addSink(
        std::make_unique<ColouredCoutSink>(false), &ColouredCoutSink::displayColouredLog);
    g3::initializeLogging(logWorker.get());

    auto log_input = std::make_unique<ThreadedProtoUdpListener<TbotsProto::RobotLog>>(
            std::string(ROBOT_MULTICAST_CHANNELS.at(0)) + "%wlp2s0",
            ROBOT_LOGS_PORT, std::function(logFromNetworking), true);


    LOG(INFO) << "Network logger listening on channel "
              << ROBOT_MULTICAST_CHANNELS.at(0) << " and interface "
              << args.interface << std::endl;

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();

    return 0;
}
