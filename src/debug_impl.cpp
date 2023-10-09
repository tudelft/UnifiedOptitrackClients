#include "cyberzoo_mocap_client.hpp"

class DebugImpl : public CyberZooMocapClient
{
private:
    /* data */
public:
    
};




int main(int argc, char const *argv[])
{
    DebugImpl client = DebugImpl();
    client.print_startup();

    return 0;
}
