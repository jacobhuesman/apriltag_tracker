#ifndef APRILTAG_TRACKER_ERRORS_H
#define APRILTAG_TRACKER_ERRORS_H

namespace apriltag_tracker
{

class unable_to_find_transform_error : public std::runtime_error
{
public:
  explicit unable_to_find_transform_error(std::string error_msg) : std::runtime_error(error_msg) {};
};

}

#endif //APRILTAG_TRACKER_ERRORS_H
