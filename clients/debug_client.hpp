#include "unified_mocap_client.hpp"

class NatNet2Debug : public UnifiedMocapClient
{
public:
    NatNet2Debug()
    {
    }

private:
    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("debug_option,d", boost::program_options::value<float>(), "just here for debugging")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("debug_option"))
        {
            float val = vm["debug_option"].as<float>();
            std::cout << "Debug option set with value "
                      << val << std::endl;
        }
    }

    void publish_data() override
    {
        /*
        more debugging output? is that even still needed, or covered by natnet2console
        */
    }
};
