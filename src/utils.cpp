#include "utils.h"

int utils::load_from_xyz(const std::string& file, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::ifstream ifs(file.c_str());
    double x, y, z;
    int result = 0;
    while(ifs >> x >> y >> z) {
        cloud->push_back(pcl::PointXYZ(x, y, z));
        result+= 1;
    }
    return result;
}

bool utils::file_date_comparator (const fs::path &p1, const fs::path &p2) {
    return fs::last_write_time(p1) < fs::last_write_time(p2);
}

int utils::parseUrgBenriXY(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::ifstream ifs(filename.c_str());
    std::string elem;
    int retval = 0;
    while( std::getline(ifs, elem, ',') ) {
        unsigned long timestamp;
        float x, y, z;
        if ( sscanf(elem.c_str(), "%lu:(%f;%f;%f)", &timestamp, &x, &y, &z) != 4 ) {
            if ( sscanf(elem.c_str(), "(%f;%f;%f)", &x, &y, &z) != 3 ) {
                return -1;
            }
        }
        ++retval;
        cloud->push_back( pcl::PointXYZ(x, y, z) );
    }
    return retval;
}
