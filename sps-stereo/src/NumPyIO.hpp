//
// Created by yaoyu on 4/8/20.
//

#ifndef NUMPYIO_HPP
#define NUMPYIO_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <cnpy.h>
#include <Eigen/Dense>

template < typename rT >
void write_depth_map_2_npy( const std::string &fn,
        const Eigen::MatrixX<rT> &mat,
        rT minLimit = 0 ) {
    const std::size_t rows = mat.rows();
    const std::size_t cols = mat.cols();
    std::vector<rT> data;

    for ( std::size_t j = 0; j < cols; ++j ) {
        for ( std::size_t i = 0; i < rows; ++i ) {
            const rT value = mat( i, j );

            if ( value > minLimit ) {
                data.push_back(static_cast<rT>(j));
                data.push_back(static_cast<rT>(i));
                data.push_back(value);
            }
        }
    }

    const std::size_t N = data.size();

    if ( 0 == N ) {
        std::stringstream ss;
        ss << "No pixels have depth over the limit of " << minLimit;
        throw( std::runtime_error( ss.str() ) );
    }

    cnpy::npy_save( fn, &data[0], { N/3, 3 }, "w" );
}

template < typename rT >
void write_eigen_matrix_2_npy( const std::string &fn,
        const Eigen::MatrixX<rT> &mat ) {
    const std::size_t rows = mat.rows();
    const std::size_t cols = mat.cols();

    if ( mat.IsRowMajor ) {
        cnpy::npy_save( fn, mat.data(), { rows, cols }, "w" );
    } else {
        Eigen::MatrixX<rT> temp = mat.transpose();
        cnpy::npy_save( fn, temp.data(), { rows, cols }, "w" );
    }
}

template < typename rT >
void read_npy_2_eigen_matrix( const std::string &fn,
        Eigen::MatrixX<rT> &mat ) {
    // Load the .npy file.
    cnpy::NpyArray arr = cnpy::npy_load(fn);

    rT* data = arr.data<rT>();

    assert( 2 == arr.shape.size() );

    const int dim0 = arr.shape[0];
    const int dim1 = arr.shape[1];

    mat.resize( dim0, dim1 );

    Eigen::Matrix<rT, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> temp =
            Eigen::Map< Eigen::Matrix<rT, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > (
            data, dim0, dim1 );

    // Convert the row major matrix to column major?
    mat = temp;
}

#endif //NUMPYIO_HPP
