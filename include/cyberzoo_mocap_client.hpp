#ifndef H_CYBERZOO_OPTITRACK_CLIENT
#define H_CYBERZOO_OPTITRACK_CLIENT

class CyberZooMocapClient
{
private:
    float publish_frequency;
    std::vector<uint> streaming_ids;

protected:
    void read_po(int argc, char const *argv[]);

public:
    CyberZooMocapClient(/* args */);
    ~CyberZooMocapClient();

    void print_startup() const;
    void print_coordinate_system() const;
};

#endif //H_CYBERZOO_OPTITRACK_CLIENT