#ifndef FILESYSTEM_HPP
#define FILESYSTEM_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

std::vector<std::string> get_file_parts(const std::string& path) {
    boost::filesystem::path p(path);

    std::vector<std::string> parts;

    parts.push_back( p.parent_path().string() );
    parts.push_back( p.stem().string() );
    parts.push_back( p.extension().string() );

    if ( 0 == parts[0].length() ) {
        parts[0] = "./";
    }

    return parts;
}

void test_directory(const std::string& dir) {
    boost::filesystem::path p(dir);

    if ( !boost::filesystem::is_directory(p) ) {
        // Create the directory.
        try {
            boost::filesystem::create_directories(p);
        } catch ( boost::filesystem::filesystem_error& err ) {
            std::stringstream ss;
            ss << "Create directory " << dir << " failed. ";
            throw(std::runtime_error(ss.str()));
        }
    }
}

void test_directory_by_filename( const std::string& fn ) {
    // Get the file parts.
    auto parts = get_file_parts(fn);

    // Test the output directory.
    test_directory( parts[0] );
}

bool test_file( const std::string &fn ) {
    boost::filesystem::path p(fn);

    return boost::filesystem::is_regular_file(p);
}

#endif //FILESYSTEM_HPP
