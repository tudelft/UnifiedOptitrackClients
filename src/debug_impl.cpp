#include "cyberzoo_mocap_client.hpp"

class DebugImpl : public CyberZooMocapClient
{
public:
    DebugImpl(int argc, char const *argv[]) : CyberZooMocapClient(argc, argv)
    {
    }

private:
    
};




int main(int argc, char const *argv[])
{
    DebugImpl client = DebugImpl(argc, argv);
    return 0;
}
