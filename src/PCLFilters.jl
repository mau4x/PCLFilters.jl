module PCLFilters

export AbstractFilter, getRemovedIndices,
    AbstractVoxelGridFilter, UniformSampling, PassThrough, VoxelGrid,
    ApproximateVoxelGrid, StatisticalOutlierRemoval, RadiusOutlierRemoval,
    ExtractIndices,
    setRadiusSearch, setFilterFieldName, setFilterLimits, setLeafSize,
    setKeepOrganized, setMeanK, setStddevMulThresh, setMinNeighborsInRadius,
    setNegative

using LibPCL
using PCLCommon
using Cxx

const libpcl_filters = LibPCL.find_library_e("libpcl_filters")
try
    Libdl.dlopen(libpcl_filters, Libdl.RTLD_GLOBAL)
catch e
    warn("You might need to set DYLD_LIBRARY_PATH to load dependencies proeprty.")
    rethrow(e)
end

cxx"""
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
"""

import Base: filter

abstract AbstractFilter <: PCLBase
abstract AbstractVoxelGridFilter <: AbstractFilter

### Abstract methods for Filter ###

function getRemovedIndices(f::AbstractFilter)
    icxx"$(f.handle)->getRemovedIndices();"
end
filter(f::AbstractFilter, cloud::cxxt"boost::shared_ptr<std::vector<int>>") =
    icxx"$(f.handle)->filter(*$(cloud));"
filter(f::AbstractFilter, cloud::PointCloud) =
    icxx"$(f.handle)->filter(*$(cloud.handle));"

### Filter types ###

for (name, supername) in [
    (:UniformSampling, AbstractFilter),
    (:PassThrough, AbstractFilter),
    (:VoxelGrid, AbstractVoxelGridFilter),
    (:ApproximateVoxelGrid, AbstractVoxelGridFilter),
    (:StatisticalOutlierRemoval, AbstractFilter),
    (:RadiusOutlierRemoval, AbstractFilter),
    (:ExtractIndices, AbstractFilter),
    ]
    cxxname = "pcl::$name"
    valname = symbol(name, "Val")
    @eval begin
        @defpcltype $name{T} <: $supername $cxxname
        @defptrconstructor $name{T}() $cxxname
        @defconstructor $valname{T}() $cxxname
    end
end

setRadiusSearch(us::UniformSampling, ss) =
    icxx"$(us.handle)->setRadiusSearch($ss);"

setFilterFieldName(pass::PassThrough, name::AbstractString) =
    icxx"$(pass.handle)->setFilterFieldName($(pointer(name)));"
setFilterLimits(pass::PassThrough, lo, hi) =
    icxx"$(pass.handle)->setFilterLimits($lo, $hi);"

for f in [:setKeepOrganized, :setFilterLimitsNegative]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(pass.handle)->$f(\$v);")
    @eval $f(pass::PassThrough, v::Bool) = $body
end

setLeafSize(v::AbstractVoxelGridFilter, lx, ly, lz) =
    icxx"$(v.handle)->setLeafSize($lx, $ly, $lz);"


for f in [:setMeanK, :setStddevMulThresh]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(s.handle)->$f(\$v);")
    @eval $f(s::StatisticalOutlierRemoval, v) = $body
end

for f in [:setRadiusSearch, :setMinNeighborsInRadius]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(r.handle)->$f(\$v);")
    @eval $f(r::RadiusOutlierRemoval, v) = $body
end

for f in [:setNegative]
    body = Expr(:macrocall, symbol("@icxx_str"), "\$(ex.handle)->$f(\$v);")
    @eval $f(ex::ExtractIndices, v) = $body
end

end # module
