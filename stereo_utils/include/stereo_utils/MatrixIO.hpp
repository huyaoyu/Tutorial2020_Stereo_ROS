//
// Created by yaoyu on 3/29/20.
//

#ifndef MATRIXIO_HPP
#define MATRIXIO_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <Eigen/Dense>

template < typename rT >
void read_matrix( const std::string& fn, int rows, int cols, const std::string& delimiter,
        Eigen::MatrixX<rT>& mat ) {
    assert(rows > 0);
    assert(cols > 0);

    mat.resize(rows, cols);

    std::string line;
    std::ifstream ifs(fn);
    std::size_t count = 0;

    if ( ifs.is_open() ) {
        while( std::getline(ifs, line) ) {
            boost::algorithm::trim(line);

            if ( line.empty() ) {
                continue;
            }

            if ( count == rows ) {
                std::stringstream ss;
                ss << "To many rows. Expecting " << rows << " rows. ";
                throw( std::runtime_error( ss.str() ) );
            }

            std::vector<std::string> splitStrings;

            boost::split( splitStrings, line, boost::is_any_of(delimiter) );

            if ( splitStrings.size() != cols ) {
                std::stringstream ss;
                ss << "Line " << count << " contains tokens other than " << cols << ". "
                   << "Line: " << line;
                throw( std::runtime_error( ss.str() ) );
            }

            for ( int i = 0; i < splitStrings.size(); ++i ) {
                std::stringstream ss( splitStrings[i] );
                rT number;
                ss >> number;

                mat(count, i) = number;
            }

            count++;
        }
    } else {
        std::stringstream ss;
        ss << "File " << fn << " not opened.";
        throw(std::runtime_error(ss.str()));
    }

    ifs.close();
}

template < typename Derived >
void write_matrix( const std::string &fn,
        const Eigen::MatrixBase<Derived> &mat,
        const std::string &delimiter="," ) {

    std::ofstream ofs(fn);

    if ( !ofs.good() ) {
        std::stringstream ss;
        ss << fn << " not good. ";
        throw std::runtime_error(ss.str());
    }

    const Eigen::IOFormat CSVFormat(
            Eigen::StreamPrecision, Eigen::DontAlignCols,
            delimiter, "\n");

    ofs << mat.format(CSVFormat);

    ofs.close();
}

#endif //MATRIXIO_HPP
