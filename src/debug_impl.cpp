#include "cyberzoo_mocap_client.hpp"

class DebugImpl : public CyberZooMocapClient
{
public:
    DebugImpl()
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
        for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        {
            pose_t pose = this->getPoseRB(i);
            pose_der_t pose_der = this->getPoseDerRB(i);

            printf("Timestamp pose: \t%ldus\n", pose.timeUs);
            printf("Timestamp pose_der: \t%ldus\n", pose_der.timeUs);
            printf("Rigid Body [ID=%d Valid=%d]\n", this->getIdRB(i), this->getValidRB(i));
		    printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		    printf("\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
		    	pose.x, pose.y, pose.z,
		    	pose.qx, pose.qy, pose.qz, pose.qw);
            printf("\tvx\tvy\tvz\twx\twy\twz\n");
		    printf("\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
                pose_der.x, pose_der.y, pose_der.z, pose_der.wx, pose_der.wy, pose_der.wz);
        }
    }
};

int main(int argc, char const *argv[])
{
    DebugImpl client = DebugImpl();
    client.start(argc, argv);
    return 0;
}
