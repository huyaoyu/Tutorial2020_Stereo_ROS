#ifndef JSONHELPER_HPP
#define JSONHELPER_HPP

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "nlohmann/json.hpp"

using JSON = nlohmann::json;

namespace stereo_utils
{

std::shared_ptr<JSON> read_json( const std::string &fn );

} // namespace stereo_utils

#endif //JSONHELPER_HPP
