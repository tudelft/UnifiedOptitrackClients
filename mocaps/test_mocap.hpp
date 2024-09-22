// Copyright 2024 Till Blaha (Delft University of Technology)
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.

#ifndef H_TEST_MOCAP
#define H_TEST_MOCAP

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>

#include <iostream>
#include <csignal>
#include <thread>

#include <unistd.h>
#include <termios.h>

namespace po = boost::program_options;

class TestMocap : public Mocap
{
public:
    TestMocap() : Mocap{}, stopFlag{false}, freq{10.}
    {
        // setup work, but do as little as possible
    }

    ~TestMocap()
    {
        // cleanup if necessary
        stop_sending_data();
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##  _____       _    ##
## |_   _|__ __| |_  ##
##   | |/ -_|_-<  _| ##
##   |_|\___/__/\__| ##
#######################)";
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        // add extra commandline options if you need to.
        // avoid those already used in UnifiedMocapClient::read_po()
        //desc.add_options()
        //    ("dontmindme,d", boost::program_options::value<std::string>(), "Optimal extra argument for demonstration purposes")
        //    ("listofint,i", boost::program_options::value<std::vector<unsigned int>>()->multitopken(), "Optional list of values for demonstration purposes")
        //;
        desc.add_options()
            ("freq", po::value<float>(), "Test Mocap new sample frequency")
            ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        /*
        if (vm.count("dontmindme")) {
            // value can be accessed with vm["dontmindme"].as<std::string>();
            std::cout << "Give user some feedback about the chosen" << vm["dontmindme"].as<std::string>() << std::endl;
        } else {
            // fail if not found, or use some default value
            std::cout << "dontmindme not specified. aborting..." << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("listofint")) {
            // values can be accessed via vm["listofint"].as<std::vector<unsigned int>>();
        } else {
            // do something
        }
        */

        if (vm.count("freq") == 1) {
            this->freq = vm["freq"].as<float>();
            this->freq = (this->freq > 1.f) ? this->freq : 1.f; // limit to 1Hz min
        } else {
            std::cout << "--freq must be given exactly once." << std::endl;
            std::raise(SIGINT);
        }

        // simulate tracking of one RB
        this->trackRB(0);
    }

    int connect() override
    {
        // will be called the first thing after all options are parsed
        // should be used to establish connection with the MOCAP system, and
        // configure it if necessary

        // IMPORTANT: 
        // every time a new sample comes in, the base class method 
        // "processNewPose" must be invoked.
        // in this test example, we do this at the end of the send_test_data
        // thread.
        // also see the optitrack client, which does it with callback
        testDataSender = std::thread(&TestMocap::send_test_data,
                                    this,
                                    1000.f / this->freq // 10Hz
                                    );
        return 0;
    }

    // Simply a method send test data to the pipeline
    void send_test_data(int intervalMs) {
        while (!stopFlag) {
            pose_t newPose {
                0,
                1.f, 2.f, 3.f,      // position
                0.f, 0.f, 0.f, 1.f  // quaternion (x,y,z,w)
            };

            this->processNewPose(0, newPose);

            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }

    void stop_sending_data(void) {
        stopFlag = true;
        if (testDataSender.joinable()) {
            testDataSender.join();
        }
    }
private:
    std::thread testDataSender;
    bool stopFlag;
    float freq;
};

#endif // H_MOCAP