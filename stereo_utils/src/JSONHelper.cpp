#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include "JSONHelper.hpp"

using JSON = nlohmann::json;

using namespace stereo_utils;

std::shared_ptr<JSON> stereo_utils::read_json( const std::string &fn ) {
    std::shared_ptr<JSON> pJson ( new JSON );

    std::ifstream ifs(fn);

    if ( !ifs.good() ) {
        std::stringstream ss;
        ss << fn << " not good. ";
        throw std::runtime_error( ss.str() );
    }

    ifs >> *pJson;

    return pJson;
}
