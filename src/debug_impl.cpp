#include "cyberzoo_mocap_client.hpp"

class DebugImpl : public CyberZooMocapClient
{
public:
    DebugImpl(int argc, char const *argv[]) : CyberZooMocapClient(argc, argv)
    {
    }

    void listenToKeystrokes()
    {
        // wait for keystrokes
        std::cout << std::endl << "Listening to messages! Press q to quit, Press t to toggle message printing" << std::endl;
    	while ( const int c = getch() )
        {
            switch(c)
            {
                case 'q':
                    delete this;
                    break;
                case 't':
                    this->togglePrintMessages();
                    break;
            }
        }
    };

private:
    
};




int main(int argc, char const *argv[])
{
    DebugImpl client = DebugImpl(argc, argv);

    if (!client.isInitialized())
        return 1;

    client.listenToKeystrokes();

    return 0;
}
