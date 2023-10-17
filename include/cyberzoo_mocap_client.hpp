#ifndef H_CYBERZOO_OPTITRACK_CLIENT
#define H_CYBERZOO_OPTITRACK_CLIENT

#include <vector>


enum CoordinateSystem { UNCHANGED=0, NED, ENU};

class CyberZooMocapClient 
{
private:
    float publish_frequency;
    std::vector<unsigned int> streaming_ids;
    CoordinateSystem co;

    void read_po(int argc, char const *argv[]);

    void print_startup() const;
    void print_coordinate_system() const;

protected:
   

public:
    CyberZooMocapClient(int argc, char const *argv[]);
    ~CyberZooMocapClient();
};

#endif //H_CYBERZOO_OPTITRACK_CLIENT