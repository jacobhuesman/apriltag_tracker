#ifndef PROJECT_ERRORS_H
#define PROJECT_ERRORS_H

namespace AprilTagTracker
{

class unable_to_find_transform_error : public std::runtime_error
{
public:
  explicit unable_to_find_transform_error(std::string error_msg) : std::runtime_error(error_msg) {};
};

}

#endif //PROJECT_ERRORS_H
