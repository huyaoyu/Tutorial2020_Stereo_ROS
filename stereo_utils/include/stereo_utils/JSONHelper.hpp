#ifndef JSONHELPER_HPP
#define JSONHELPER_HPP

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "nlohmann/json.hpp"

using JSON = nlohmann::json;

static std::shared_ptr<JSON> read_json( const std::string &fn ) {
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

#endif //JSONHELPER_HPP
