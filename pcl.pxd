# -*- coding: utf-8 -*-

from libcpp cimport bool
from boost_shared_ptr cimport shared_ptr

cdef extern from "pcl/point_types.h" namespace "pcl" nogil:
    cdef struct PointXYZ:
        PointXYZ()
        float x
        float y
        float z

cdef extern from "pcl/point_cloud.h" namespace "pcl" nogil:
    cdef cppclass PointCloud[T]:
        PointCloud() except +
        PointCloud(unsigned int, unsigned int) except +
        unsigned int width
        unsigned int height
        bool is_dense
        void resize(size_t) except +
        size_t size()
        T& at(size_t) except +
        shared_ptr[PointCloud[T]] makeShared()

ctypedef PointCloud[PointXYZ] PointCloud_t
ctypedef shared_ptr[PointCloud[PointXYZ]] ShPtr_PointCloud_t

cdef extern from "ndt.h" nogil:
    cdef cppclass NDTSettings:
        int iter
        double grid_step
        double grid_extent
        double optim_step_x
        double optim_step_y
        double optim_step_theta
        double epsilon
        double guess_x
        double guess_y
        double guess_theta
