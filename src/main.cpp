#include <boost/filesystem.hpp>

#include "cyberzoo_mocap_client.hpp"
#include "debug_client.hpp"

#ifdef USE_CLIENT_IVY
    #include "ivy_client.hpp"
#endif

int main(int argc, char const *argv[])
{
    boost::filesystem::path p(argv[0]);

#ifdef USE_CLIENT_DEBUG
    if (p.filename() == "natnet2debug") {
        std::cout << "Using client " << p.filename() << std::endl;

        DebugImpl client = DebugImpl();
        client.start(argc, argv);
    } else
#endif
#ifdef USE_CLIENT_IVY
    if (p.filename() == "natnet2ivy") {
        std::cout << "Using client " << p.filename() << std::endl;

        NatNet2Ivy client = NatNet2Ivy();
        client.start(argc, argv);
    } else 
#endif
    {
        std::cout << "Support for client " << p.filename() << "was not compiled into the program." << std::endl;
        return 1;
    }

    return 0;
}

